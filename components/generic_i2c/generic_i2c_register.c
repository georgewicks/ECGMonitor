/**
 * @file generic_i2c_register.c
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief This set of functions was derived from the PCA9685 LED PWM Driver
 * @version 0.1
 * @date 2025-05-07
 */
/*************************************************** 
  This is a library for the PCA9685 LED PWM Driver

  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "esp_err.h"
#include <errno.h>
#include "esp_log.h"
#include "esp_system.h"
#include "generic_i2c_register.h"

/**
 * @brief      Write a 16 bit value to a register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param[in]  value    The value
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register_word(int8_t dev_addr, uint8_t regaddr, uint16_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN));

    // swap bytes issue?
    //ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value & 0xff, ACK_VAL));
    //ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value >> 8, NACK_VAL));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value >> 8, ACK_VAL));           // MSB firsst

    // ?? Examples in ESP-IDF docs seem to indicate that we do NOT put out the NACK for the last
    // byte, BUT instead ACK_VAL
    //ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value & 0xff, NACK_VAL));        // LSB last!
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value & 0xff, ACK_VAL));        // LSB last!

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief      Write two 16 bit values to the same register on an i2c device
 *
 * @param[in]  regaddr   The register address
 * @param[in]  value1   The value on
 * @param[in]  value2  The value off
 * 
 * @return     result of command
 */
esp_err_t generic_write_i2c_register_two_words(uint8_t dev_addr, uint8_t regaddr, uint16_t value1, uint16_t value2)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value1 & 0xff, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value1 >> 8, NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value2 & 0xff, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value2 >> 8, NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief      Read two 8 bit values from the same register on an i2c device
 *
 * @param[in]  regaddr  The register address
 * @param      valueA   The first value
 * @param      valueB   The second value
 *
 * @return     result of command
 */
esp_err_t generic_read_two_i2c_register(uint8_t dev_addr, uint8_t regaddr, uint8_t* valueA, uint8_t* valueB)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, dev_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, valueA, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, valueB, NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief      Read a 16 bit value from a register on an i2c decivde
 *
 * @param[in]  regaddr  The register address
 * @param      value    The value
 *
 * @return     result of command
 */
esp_err_t generic_read_i2c_register_word(uint8_t dev_addr, uint8_t regaddr, uint16_t* value)
{
    esp_err_t ret;

    uint8_t valueA;
    uint8_t valueB;

    ret = generic_read_two_i2c_register(dev_addr, regaddr, &valueA, &valueB);
    if (ret != ESP_OK) {
        return ret;
    }

    //*value = (valueB << 8) | valueA;          endianness
    *value = (valueA << 8) | valueB;

    return ret;
}

