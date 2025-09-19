#include <stdio.h>
#include <ECGcommon.h>
#include <sdcard.h>
#include <ECGSignalCapture.h>
#include <InitI2C.h>

static  char    *TAG="appmain";

const char *ECG_file = MOUNT_POINT"/ECG.bin";

void     Do_I2C_scan_for_devices(void);

/**
 * @brief fatal error handler
 * 
 */
void    fatal_error(void)
{
    ESP_LOGI(TAG, "Fatal error");
    abort();
}

// esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time)

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

    Do_I2C_scan_for_devices();

    for(;;)
    {
        vTaskDelay(1000/ portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"+");
    } 

}

#define     MAX_OUTPUT_TXT          (4096)
#define     BRA_I2C_MAX_NUM         (128)
typedef struct _BRA_I2C_dev_
{
    uint8_t     I2C_dev_addr;
    void*       ptr_Dev_API_Parm;       // TBD
} BRA_I2C_dev;

char        output_text[MAX_OUTPUT_TXT];
uint8_t     BRA_I2C_LUT[BRA_I2C_MAX_NUM];
typedef struct _BRA_I2C_List_
{
    int16_t     num_devices;
    BRA_I2C_dev  i2c_list[BRA_I2C_MAX_NUM];
} BRA_I2C_List;

i2c_config_t        BRA_I2C_conf;            // for now, global
BRA_I2C_List        BRA_I2C_list;

/**
 * @brief Do_I2C_scan_for_devices() - This function will query the I2C bus for the
 * existence of an I2C device at a given address. 
 * 
 * @return void
 */
void     Do_I2C_scan_for_devices(void)
{
    esp_err_t res;

    ESP_LOGI(TAG,"%s: called",__func__);

    char messsage[256];
    memset(output_text,'\0',sizeof(output_text));

    strcat(output_text, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    strcat(output_text,"00:         ");
    //ESP_LOGI(Tag,"     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    //ESP_LOGI(Tag,"00:         ");
    //for (uint8_t i = 3; i < 0x78; i++)

    for (uint8_t i = 0; i < 0x78; i++)
    {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if(cmd == NULL)
        {
            ESP_LOGE(TAG, "%s: i2c_cmd_link_create failed!", __func__);
            return;
        }
        i2c_master_start(cmd);
        vTaskDelay(pdMS_TO_TICKS(15));
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

        if(res == ESP_OK)
        {
            BRA_I2C_LUT[i] = 1;
        }

        // Problem: the ESP_LOGx functions will always output a newline after each call,
        // which will ruin the nice 16 column format, with each column entry - need to go back
        // to the printf calls.. 
        if (i % 16 == 0)
        {
            // address prefix
            //ESP_LOGI(Tag,"\n%.2x:", i);
            sprintf(messsage,"\n%.2x:", i );
            strcat(output_text,messsage);
            //printf("\n%.2x:", i);
        }
        if (res == 0)
        {
            //ESP_LOGI(Tag," %.2x", i);
            //printf(" %.2x", i);
            sprintf(messsage," %.2x:", i );
            strcat(output_text,messsage);
        }
        else if (res == ESP_ERR_TIMEOUT)
        {
            ESP_LOGI(TAG,"UU ");
            //printf("UU ");
            sprintf(messsage," UU" );
            strcat(output_text,messsage);
        }
        else
        {
            //ESP_LOGI(Tag," --");
            //printf(" --");
            sprintf(messsage," --" );
            strcat(output_text,messsage);
       }

        if ((i % 16 == 0) || (res == 0))
        {
            int ii;
            ii = BRA_I2C_list.num_devices;
            BRA_I2C_list.i2c_list[ii].I2C_dev_addr = i;
            BRA_I2C_list.num_devices++;
        }

        i2c_cmd_link_delete(cmd);
    }
    // Finally, at the end of the scan we output the I2c devices we found.
    ESP_LOGI(TAG,"\n%s",output_text);
    return;
}
