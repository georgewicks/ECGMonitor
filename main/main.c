#include <stdio.h>
#include <ECGcommon.h>
#include <sdcard.h>
#include <ECGSignalCapture.h>
#include <InitI2C.h>

static  char    *TAG="appmain";

const char *ECG_file = MOUNT_POINT"/ECG.bin";

/**
 * @brief fatal error handler
 * 
 */
void    fatal_error(void)
{
    ESP_LOGI(TAG, "Fatal error");
    abort();
}

esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time)

void app_main(void)
{
    ESP_LOGI(TAG, "%s called", __func__ );

    esp_err_t ret = InitSDCard();

    // Find the RTC & enable ECG signal capture

    ret = sigdata_open_ECG_file(ECG_file);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to initialize ECG_file! = %s", esp_err_to_name(ret));
        fatal_error();
    }

    vTaskDelay(1000/ portTICK_PERIOD_MS);

    // Initialize I2C driver
    InitI2C(I2C_NUM_0);

    // The I2C devices may need to stretch out responses, so allow for larger timeout.
    i2c_set_timeout(I2C_NUM_0, 100000);

}