/**
 * @file InitI2C.h
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma    once
 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "ECGcommon.h"

//#include "common.h"

// Note: there may be more than one I2C number, particularly if the use of the I2C bus 
// extender is required.
extern void    InitI2C(int I2C_number);
