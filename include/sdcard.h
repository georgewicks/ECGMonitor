/**
 * @file sdcard.h
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief   Header file for the SD card module
 * @version 0.1
 * @date 2025-07-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "sdmmc_cmd.h"
#include "driver/sdmmc_types.h"

#define MOUNT_POINT "/sdcard"

#define SZ_860_SAMP     (860)
typedef struct _ECGRec_ {
    uint16_t    marker;
    uint8_t     timestamp[8];  
    uint16_t    databuff[SZ_860_SAMP];
} ECGRec;

// basic API functions
esp_err_t   sigdata_open_ECG_file(const char *path);
esp_err_t   sigdata_write_datarec2file(void *ptr);
void        sigdata_close_ECG_file(void);

// Initialize the sdcard interface
esp_err_t   InitSDCard(void);

