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
/*************************************************** 
  This is a library for the PCA9685 LED PWM Driver

  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/
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


