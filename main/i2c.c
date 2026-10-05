#include "i2c.h"
#include "driver/i2c_master.h"
#include "lsm6dsox_reg.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <inttypes.h>

float acc_16g_sensitivity = 0.488;
float gyro_2000dps_sensitivity = 70;

char * sensor_names[6] = {"gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"};

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t imu_handle;
static const uint8_t IMU_ADDR = 0x6A;
uint32_t timeout = 50;

static const uint8_t ctrl3_c_addr = LSM6DSOX_CTRL3_C;
static lsm6dsox_reg_t ctrl3_c_value;

static const uint8_t status_reg_addr = LSM6DSOX_STATUS_REG;
static lsm6dsox_reg_t status_reg_value;

static const uint8_t who_am_i_addr = LSM6DSOX_SPI2_WHO_AM_I;

static const uint8_t ctrl1_xl_addr = LSM6DSOX_CTRL1_XL;
static lsm6dsox_reg_t ctrl1_x1_value;

static const uint8_t ctrl8_xl_addr = LSM6DSOX_CTRL8_XL;
static lsm6dsox_reg_t ctrl8_xl_value;

static const uint8_t ctrl2_g_addr = LSM6DSOX_CTRL2_G;
static lsm6dsox_reg_t ctrl2_g_value;

static const uint8_t outx_l_g_addr = LSM6DSOX_OUTX_L_G;

void print_bits(const char *label, uint8_t value){
    char bits[10];  // 8 bits + 1 space + '\0'
    int pos = 0;

    for (int i = 7; i >= 0; i--) {
        bits[pos++] = (value & (1 << i)) ? '1' : '0';
        if (i == 4) {
            bits[pos++] = ' ';
        }
    }
    bits[pos] = '\0';

    printf("%s: 0x%02X  %s\n", label, value, bits);
}

esp_err_t init_bus(){
	
	i2c_master_bus_config_t i2c_mst_config = {
	    .clk_source = I2C_CLK_SRC_DEFAULT,
	    .i2c_port = -1,
	    .scl_io_num = 22,
	    .sda_io_num = 21,
	    .glitch_ignore_cnt = 7,
	    .flags.enable_internal_pullup = false,
	};
	
	return i2c_new_master_bus(&i2c_mst_config, &bus_handle);
}

esp_err_t init_imu(){

	i2c_device_config_t imu_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = IMU_ADDR,
	    .scl_speed_hz = 100000,
	};
	return i2c_master_bus_add_device(bus_handle, &imu_cfg, &imu_handle);
}


esp_err_t probe_bus(uint8_t addr){
	
	return i2c_master_probe(bus_handle, addr, timeout);
}

esp_err_t write_byte_to_reg(uint8_t reg_addr, uint8_t value){
	
	uint8_t write_buffer[2] = {reg_addr, value};
	
	return i2c_master_transmit(imu_handle, write_buffer, 2, timeout);
}

esp_err_t read_bytes_from_reg(const uint8_t *reg_addr, uint8_t *read_buffer, uint8_t read_buffer_len){
	
	return i2c_master_transmit_receive(imu_handle, reg_addr, 1, read_buffer, read_buffer_len, timeout);
}

esp_err_t read_who_i_am(uint8_t * read_buffer, uint8_t len){
	
	ESP_ERROR_CHECK(init_imu());
	
	return read_bytes_from_reg(&who_am_i_addr, read_buffer, len);
}

int configure_IMU(){

	esp_err_t ret;
	
	ret = read_bytes_from_reg(&ctrl3_c_addr, &ctrl3_c_value.byte, 1);
	//print_bits("ctrl3_c_value1", ctrl3_c_value.byte);
	ctrl3_c_value.ctrl3_c.sw_reset = 1;
	//print_bits("ctrl3_c_value2", ctrl3_c_value.byte);
	ret = write_byte_to_reg(ctrl3_c_addr, ctrl3_c_value.byte);
	for(;;){
		ret = read_bytes_from_reg(&ctrl3_c_addr, &ctrl3_c_value.byte, 1);
		if(ctrl3_c_value.ctrl3_c.sw_reset != 1){
			break;
		}
	}
	//print_bits("ctrl3_c_value3", ctrl3_c_value.byte);
	ctrl3_c_value.ctrl3_c.bdu = 1; 
	//print_bits("ctrl3_c_value4", ctrl3_c_value.byte);
	ret = write_byte_to_reg(ctrl3_c_addr, ctrl3_c_value.byte);
	
	ret = read_bytes_from_reg(&ctrl8_xl_addr, &ctrl8_xl_value.byte, 1);
	//print_bits("ctrl8_xl_value1", ctrl8_xl_value.byte);
	ctrl8_xl_value.ctrl8_xl.xl_fs_mode = 0; // enable the FS of acc to reach 16 g
	//print_bits("ctrl8_xl_value2", ctrl8_xl_value.byte);
	ret = write_byte_to_reg(ctrl8_xl_addr, ctrl8_xl_value.byte);
	
	ret = read_bytes_from_reg(&ctrl1_xl_addr, &ctrl1_x1_value.byte, 1);
	//print_bits("ctrl1_x1_value", ctrl1_x1_value.byte);
	ctrl1_x1_value.ctrl1_xl.odr_xl = 0b0100; // 104 Hz
	ctrl1_x1_value.ctrl1_xl.fs_xl = 0b01; // 16 g
	//print_bits("ctrl1_x1_value2", ctrl1_x1_value.byte);
	ret = write_byte_to_reg(ctrl1_xl_addr, ctrl1_x1_value.byte);
	
	ret = read_bytes_from_reg(&ctrl2_g_addr, &ctrl2_g_value.byte, 1);
	//print_bits("ctrl2_g_value", ctrl2_g_value.byte);	
	ctrl2_g_value.ctrl2_g.odr_g = 0b0100; // 104 Hz
	ctrl2_g_value.ctrl2_g.fs_g = 0b110; // 2000 dps
	//print_bits("ctrl2_g_value2", ctrl2_g_value.byte);	
	ret = write_byte_to_reg(ctrl2_g_addr, ctrl2_g_value.byte);
	
	
	return ret;
	
}


void print_sensor_values(){
	
	uint8_t read_buffer[12] = {0};
	float f_sensor_buffer[6] = {0};
	
	read_bytes_from_reg(&outx_l_g_addr, read_buffer, 12);
	
	for(uint8_t i = 0; i < 6; i++){
		float sensitivity = i < 3 ? gyro_2000dps_sensitivity : acc_16g_sensitivity;
		f_sensor_buffer[i] = (int16_t)((read_buffer[i*2+1] << 8) | read_buffer[i*2]) * sensitivity * (float)0.001;
		printf("%s%s: %9.3f ", i == 0 ? "\r" : "", sensor_names[i], f_sensor_buffer[i]);
	}
	fflush(stdout);
	
}

void transmit_sensor_csv(void *pvParameters){
	
	uint8_t read_buffer[12] = {0};
	float f_sensor_buffer[6] = {0};
	int64_t time_ms;
	
	while(true){
		read_bytes_from_reg(&status_reg_addr, &status_reg_value.byte, 1);
		if(status_reg_value.status_reg.gda && status_reg_value.status_reg.xlda){
			
			read_bytes_from_reg(&outx_l_g_addr, read_buffer, 12);
				
			for(uint8_t i = 0; i < 6; i++){
				float sensitivity = i < 3 ? gyro_2000dps_sensitivity : acc_16g_sensitivity;
				f_sensor_buffer[i] = (int16_t)((read_buffer[i*2+1] << 8) | read_buffer[i*2]) * sensitivity * (float)0.001;
			}
			time_ms = esp_timer_get_time() / 1000;

				printf("%" PRId64 ",%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",time_ms, f_sensor_buffer[0], f_sensor_buffer[1], f_sensor_buffer[2], f_sensor_buffer[3], f_sensor_buffer[4], f_sensor_buffer[5]);
		}
		else {
			vTaskDelay(1);
		}

	}
}


