/**
 * @file ECGSignalCapture.c
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief  Workhorse for the capture of the ECG data.
 * @version 0.1
 * @date 2025-06-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "ECGSignalCapture.h"

#define RDY_PIN         5

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



static const char *TAG = "ECG_SignalCap";

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

// extern ads1115_t        ads1115_cfg;
void                    gpio_pad_select_gpio(uint8_t gpio_num);
void                    ESP32ECG_sampler_task(void *arg);
void                    OLD_ESP32ECG_sampler_task( void *arg);

// GPIO RDY items
gpio_config_t io_conf = {};
bool    bRDY_PIN_AVAIL = false;         // RDY pin flag
#define INPUT_PIN   (5)


#define     MAX_ECG_SAMPLES     2048

uint16_t                ECG_index = 0;
uint16_t                ECG_sample_buffer[MAX_ECG_SAMPLES];
static QueueHandle_t    gpio_evt_queue = NULL;


static void IRAM_ATTR alrtpin_isr_handler(void* arg)
{
    //uint32_t gpio_num = (uint32_t) arg;
    bRDY_PIN_AVAIL = true;
    char    gpio_num = 'I';
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); 
}

void    InitI2C(void)
{
    esp_err_t   retval;

    // Setup I2C
    retval = i2c_param_config(I2C_NUM, &i2c_cfg);
    if(retval != ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_param_config failed = %s\n", esp_err_to_name(retval));
        abort();
    }
    else
        ESP_LOGI(TAG, "i2c_param_config  ok");
    retval = i2c_driver_install(I2C_NUM, I2C_MODE, I2C_RX_BUF_STATE, I2C_TX_BUF_STATE, I2C_INTR_ALOC_FLAG);
    if(retval != ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_driver_install failed = %s\n", esp_err_to_name(retval));
        abort();
    }
    else
        ESP_LOGI(TAG, "i2c_driver_install  ok");

}

void    dummy_start()
{

     //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

#if 0
    gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);
#endif

   
}