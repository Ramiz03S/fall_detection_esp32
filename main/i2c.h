/*
 * i2c.h
 *
 *  Created on: Oct 2, 2026
 *      Author: ramiz
 */

#ifndef MAIN_I2C_H_
#define MAIN_I2C_H_

#include <stdint.h>
#include "esp_err.h"

esp_err_t init_bus();
esp_err_t probe_bus(uint8_t addr);
esp_err_t write_byte_to_reg(uint8_t reg_addr, uint8_t value);
esp_err_t read_bytes_from_reg(const uint8_t *reg_addr, uint8_t *read_buffer, uint8_t read_buffer_len);
esp_err_t read_who_i_am(uint8_t * read_buffer, uint8_t len);
esp_err_t configure_IMU();
void print_sensor_values();


#endif /* MAIN_I2C_H_ */
