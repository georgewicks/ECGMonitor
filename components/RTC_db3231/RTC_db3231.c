/*
#include <stdio.h>
//#include "RTC_db3231.h"

void func(void)
{

}
 */


#include <stdio.h>
#include "driver/i2c.h"
#include "ds3231.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21

void DS3231_app_main(void) 
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    ds3231_init(I2C_NUM_0);
    ds3231_set_datetime(2023, 12, 4, 14, 30, 0);

    while (1) {
        ds3231_datetime_t now;
        ds3231_get_datetime(&now);
        printf("Time: %02d:%02d:%02d\n", now.hour, now.minute, now.second);
        printf("Date: %04d/%02d/%02d\n", now.year, now.month, now.day);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

  

  

