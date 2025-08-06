/**
 * @file InitI2C.c
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "InitI2C.h"

#define SDA_IO          (21)                      /*!< gpio number for I2C master data  */
#define SCL_IO          (22)                      /*!< gpio number for I2C master clock */

#define FREQ_HZ         (100000)                 /*!< I2C master clock frequency */
#define TX_BUF_DISABLE  (0)               /*!< I2C master doesn't need buffer */
#define RX_BUF_DISABLE  (0)               /*!< I2C master doesn't need buffer */

#define I2C_NUM             I2C_NUM_0                /*!< I2C number */
#define I2C_MODE            I2C_MODE_MASTER         /*!< I2C mode to act as */
#define I2C_RX_BUF_STATE    RX_BUF_DISABLE  /*!< I2C set rx buffer status */
#define I2C_TX_BUF_STATE    TX_BUF_DISABLE  /*!< I2C set tx buffer status */
#define I2C_INTR_ALOC_FLAG  (0)           /*!< I2C set interrupt allocation flag */

static const char *TAG = "InitI2C";

/* i2c setup ----------------------------------------- */
// Config profile for espressif I2C lib
i2c_config_t i2c_cfg = {                     
  .mode = I2C_MODE_MASTER, 
  .sda_io_num = SDA_IO,
  .scl_io_num = SCL_IO,
  .sda_pullup_en = GPIO_PULLUP_DISABLE,
  .scl_pullup_en = GPIO_PULLUP_DISABLE,
  .master.clk_speed = FREQ_HZ,
};

// ? Max number of I2C channels 
#define  MAX_I2C_NUM  (10)

bool  I2C_Registered[MAX_I2C_NUM] = {false};

void    InitI2C(int I2C_number)
{
    esp_err_t   retval;

    ESP_LOGI(TAG, "%s called. I2C_number = %d", __func__, I2C_number);

    // reject already installed requests.
    if(I2C_Registered[I2C_number] == true)
    {
        ESP_LOGI(TAG,"I2C driver already installed");
        return;     
    }

    // Setup I2C
    retval = i2c_param_config(I2C_number /* I2C_NUM */, &i2c_cfg);
    if(retval != ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_param_config failed = %s\n", esp_err_to_name(retval));
        abort();
    }
    else
        ESP_LOGI(TAG, "i2c_param_config  ok");
    retval = i2c_driver_install(I2C_number /* I2C_NUM */, I2C_MODE, I2C_RX_BUF_STATE, I2C_TX_BUF_STATE, I2C_INTR_ALOC_FLAG);
    if(retval != ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_driver_install failed = %s\n", esp_err_to_name(retval));
        abort();
    }
    else
        ESP_LOGI(TAG, "i2c_driver_install  ok");

    I2C_Registered[I2C_number] = true;

}

