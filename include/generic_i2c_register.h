/**
 * @file generic_i2c_register.h
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief Header file for the generic i2c functions
 * @version 0.1
 * @date 2025-05-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #pragma     once

//#error    TESTgeneric_i2c

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <freertos/FreeRTOS.h>
#include <driver/i2c.h>
#include "esp_err.h"
#include "esp_log.h"
//#include "esp_system.h"

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

esp_err_t generic_write_i2c_register_word(int8_t dev_addr, uint8_t regaddr, uint16_t value);
esp_err_t generic_write_i2c_register_two_words(uint8_t dev_addr, uint8_t regaddr, uint16_t value1, uint16_t value2);
esp_err_t generic_read_two_i2c_register(uint8_t dev_addr, uint8_t regaddr, uint8_t* valueA, uint8_t* valueB);
esp_err_t generic_read_i2c_register_word(uint8_t dev_addr, uint8_t regaddr, uint16_t* value);

// New: read & write more than 2 bytes! This was driven by the DS3231, which can have 7 byte data transfers
esp_err_t generic_i2c_dev_read_bytes(uint8_t dev_addr, const void *out_data, size_t out_size, void *in_data, size_t in_size);
esp_err_t generic_i2c_dev_write_bytes(uint8_t dev_addr, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)


#ifdef NEW_STUFF

 // templates copied from i2cdev.h
inline esp_err_t generic_i2c_dev_read_reg(uint8_t dev_addr, uint8_t reg, void *in_data, size_t in_size)
{
	return i2c_dev_read(dev_addr, &reg, 1, in_data, in_size);
}

inline esp_err_t generic_i2c_dev_write_reg(uint8_t dev_addr, uint8_t reg, const void *out_data, size_t out_size)
{
	return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}

#endif