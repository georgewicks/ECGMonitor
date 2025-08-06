/**
 * @file ADS1115.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #pragma once
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
//#include <stdint.h>
#include <inttypes.h>

void    Adafruit_ADS1115(void);
void    startADCReading(uint16_t mux, bool continuous);
float   computeVolts(int16_t counts);
int16_t getLastConversionResults();
