#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "esp_err.h"


void app_main(void)
{
	uint8_t read_buff[1];
	
	vTaskDelay(pdMS_TO_TICKS(10)); // boot time of imu
	
	esp_err_t err_bus = init_bus();
	const char *err_name_bus = esp_err_to_name(err_bus);
	printf("Bus status: %s\n", err_name_bus);
	
	if (err_bus == ESP_OK){
		
		for(uint8_t addr=0x08; addr <= 0x77; addr++){
			esp_err_t err_addr = probe_bus(addr);
			
			if(err_addr == ESP_OK){
				printf("Address 0x%02x returned ESP_OK\n", addr);
			}
		
		}
		
	}
	
	const char *err_name_WIM = esp_err_to_name(read_who_i_am(read_buff, 1));
	printf("WHO_I_AM status: %s\n", err_name_WIM);
	printf("WHO_I_AM Value: 0x%02x\n", read_buff[0]);
	
	configure_IMU();
	vTaskDelay(pdMS_TO_TICKS(100)); // discard 10 samples, settling time should be looked into more
	
    while (true) {
        print_sensor_values();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


