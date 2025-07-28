/**
 * @file ADS1115.c
 * @author George Wicks (george.r.wicks@gmail.com)
 * @brief   This will be replacement 
 * @version 0.1
 * @date 2025-06-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
//#include <stdint.h>
#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <driver/i2c.h>
//#include "ADS1115_WE.h"
//#include "ADS1115_config.h"
#include "ADS1115.h"
#include <generic_i2c_register.h>

static char *Tag = "ADS1115_DRVR";

// ORIG


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADS1X15_ADDRESS (0x48) ///< 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_POINTER_MASK (0x03)      ///< Point mask
#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1X15_REG_CONFIG_OS_SINGLE                                           \
  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY                                             \
  (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY                                          \
  (0x8000) ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1                                        \
  (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3                                        \
  (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3                                        \
  (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3                                        \
  (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

uint16_t MUX_BY_CHANNEL[] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ///< Single-ended AIN0
    ADS1X15_REG_CONFIG_MUX_SINGLE_1, ///< Single-ended AIN1
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ///< Single-ended AIN2
    ADS1X15_REG_CONFIG_MUX_SINGLE_3  ///< Single-ended AIN3
};                                   ///< MUX config by channel

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V                                          \
  (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE                                         \
  (0x0100) ///< Power-down single-shot mode (default)

#define ADS1X15_REG_CONFIG_RATE_MASK (0x00E0) ///< Data Rate Mask

#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1X15_REG_CONFIG_CMODE_TRAD                                          \
  (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator

#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008) ///< CPol Mask
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW                                        \
  (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI                                         \
  (0x0008) ///< ALERT/RDY pin is high when active

#define ADS1X15_REG_CONFIG_CLAT_MASK                                           \
  (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT                                         \
  (0x0000) ///< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator

#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003) ///< CQue Mask
#define ADS1X15_REG_CONFIG_CQUE_1CONV                                          \
  (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV                                          \
  (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV                                          \
  (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE                                           \
  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/** Gain settings */
typedef enum
{
  GAIN_TWOTHIRDS = ADS1X15_REG_CONFIG_PGA_6_144V,
  GAIN_ONE = ADS1X15_REG_CONFIG_PGA_4_096V,
  GAIN_TWO = ADS1X15_REG_CONFIG_PGA_2_048V,
  GAIN_FOUR = ADS1X15_REG_CONFIG_PGA_1_024V,
  GAIN_EIGHT = ADS1X15_REG_CONFIG_PGA_0_512V,
  GAIN_SIXTEEN = ADS1X15_REG_CONFIG_PGA_0_256V
} adsGain_t;

/** Data rates */
#define RATE_ADS1015_128SPS (0x0000)  ///< 128 samples per second
#define RATE_ADS1015_250SPS (0x0020)  ///< 250 samples per second
#define RATE_ADS1015_490SPS (0x0040)  ///< 490 samples per second
#define RATE_ADS1015_920SPS (0x0060)  ///< 920 samples per second
#define RATE_ADS1015_1600SPS (0x0080) ///< 1600 samples per second (default)
#define RATE_ADS1015_2400SPS (0x00A0) ///< 2400 samples per second
#define RATE_ADS1015_3300SPS (0x00C0) ///< 3300 samples per second

#define RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
#define RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
#define RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
#define RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
#define RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
#define RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
#define RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second



/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit ADS1X15 ADC breakouts.
*/
/**************************************************************************/
// ORIGINAL
/*
class Adafruit_ADS1X15 {
protected:
  // Instance-specific properties
  Adafruit_I2CDevice *m_i2c_dev; ///< I2C bus device
  uint8_t m_bitShift;            ///< bit shift amount
  adsGain_t m_gain;              ///< ADC gain
  uint16_t m_dataRate;           ///< Data rate
*/
struct adsvars {
  uint8_t m_bitShift;  ///< bit shift amount
  // was   adsGain_t m_gain;    ///< ADC gain
  uint16_t m_gain;
  uint16_t m_dataRate; ///< Data rate
} AdsVarsStrct;

/*
public:
  bool begin(uint8_t i2c_addr = ADS1X15_ADDRESS, TwoWire *wire = &Wire);
  int16_t readADC_SingleEnded(uint8_t channel);
  int16_t readADC_Differential_0_1();
  int16_t readADC_Differential_0_3();
  int16_t readADC_Differential_1_3();
  int16_t readADC_Differential_2_3();
  void startComparator_SingleEnded(uint8_t channel, int16_t threshold);
  int16_t getLastConversionResults();
  float computeVolts(int16_t counts);
  void setGain(adsGain_t gain);
  adsGain_t getGain();
  void setDataRate(uint16_t rate);
  uint16_t getDataRate();

  void startADCReading(uint16_t mux, bool continuous);

  bool conversionComplete();

private:
  void writeRegister(uint8_t reg, uint16_t value);
  uint16_t readRegister(uint8_t reg);
  uint8_t buffer[3];
};


// **
 * @brief Minimalist set of functionality to support the ECG data captured by 
 * the ADS1115 from the AD8232.
 * The functions are derived from the Adafruit_ADS1X15 library, but changed to 
 * support the ESP-IDF I2C Driver instead of the Arduino Wire library, because
 * the Arduino Wire support is incompatible with the ESP-IDF i2c driver, and 
 * we will be using more ESP-IDF components.
 * Functions re-done:
 * Adafruit_ADS1115::Adafruit_ADS1115() --- Initialization parameters
 * Adafruit_ADS1X15::startADCReading() --- start the conversion operation
 * Adafruit_ADS1X15::computeVolts() --- Compute volts for the given raw counts
 * Adafruit_ADS1X15::getLastConversionResults() --- This function reads the 
 * last conversion results without changing the config value.
 */


/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/
void    Adafruit_ADS1115(void) 
{
  AdsVarsStrct.m_bitShift = 0;
  //AdsVarsStrct.m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  AdsVarsStrct.m_gain = GAIN_ONE; /* +/-4.096V range = Gain 1  */
  AdsVarsStrct.m_dataRate = RATE_ADS1115_128SPS;
}
/*!
    @brief  Non-blocking start conversion function

    Call getLastConversionResults() once conversionComplete() returns true.
    In continuous mode, getLastConversionResults() will always return the
    latest result.
    ALERT/RDY pin is set to RDY mode, and a 8us pulse is generated every
    time new data is ready.

    @param mux mux field value
    @param continuous continuous if set, otherwise single shot
*/
void    startADCReading(uint16_t mux, bool continuous)
{
    esp_err_t   ret;
    /*
    uint16_t config = 
          ADS1X15_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
          ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
          ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
          ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)
    */

    uint16_t config = 
          ADS1X15_REG_CONFIG_CQUE_4CONV |   // Set CQUE to any value other than
                                            // None so we can use it in RDY mode
          ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
          ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
          ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

    if (continuous)
    {
        config |= ADS1X15_REG_CONFIG_MODE_CONTIN;
    }
    else
    {
        config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
    }

    // Set PGA/voltage range
    config |= AdsVarsStrct.m_gain;

    // Set data rate
    config |= AdsVarsStrct.m_dataRate;

    // Set channels
    config |= mux;

    // Set 'start single-conversion' bit
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;

    ret = generic_write_i2c_register_word(ADS1X15_ADDRESS,ADS1X15_REG_POINTER_CONFIG,config);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag,"Error from writing config reg = %s", esp_err_to_name(ret));
    }

    ret = generic_write_i2c_register_word(ADS1X15_ADDRESS,ADS1X15_REG_POINTER_HITHRESH,0x8000);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag,"Error from writing ADS1X15_REG_POINTER_HITHRESH reg = %s", esp_err_to_name(ret));
    }

    ret = generic_write_i2c_register_word(ADS1X15_ADDRESS,ADS1X15_REG_POINTER_LOWTHRESH,0x0000);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag,"Error from writing ADS1X15_REG_POINTER_LOWTHRESH reg = %s", esp_err_to_name(ret));
    }

}


/**************************************************************************/
/*!
    @brief  Compute volts for the given raw counts.

    @param counts the ADC reading in raw counts

    @return the ADC reading in volts
*/
/**************************************************************************/
float computeVolts(int16_t counts) 
{
    // see data sheet Table 3
    float fsRange;
    switch (AdsVarsStrct.m_gain)
    {
    case GAIN_TWOTHIRDS:
        fsRange = 6.144f;
        break;
    case GAIN_ONE:
        fsRange = 4.096f;
        break;
    case GAIN_TWO:
        fsRange = 2.048f;
        break;
    case GAIN_FOUR:
        fsRange = 1.024f;
        break;
    case GAIN_EIGHT:
        fsRange = 0.512f;
        break;
    case GAIN_SIXTEEN:
        fsRange = 0.256f;
        break;
    default:
        fsRange = 0.0f;
    }
    return ((uint16_t)counts) * (fsRange / (32768 >> AdsVarsStrct.m_bitShift));
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

    @return the last ADC reading
*/
/**************************************************************************/
int16_t getLastConversionResults() {
    // Read the conversion results
    // uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT) >> m_bitShift;
    uint16_t res;
    esp_err_t e = generic_read_i2c_register_word(ADS1X15_ADDRESS, ADS1X15_REG_POINTER_CONVERT, &res);

    if (AdsVarsStrct.m_bitShift == 0)
    {
        return (int16_t)res;
    }
    else
    {
        // Shift 12-bit results right 4 bits for the ADS1015,
        // making sure we keep the sign bit intact
        if (res > 0x07FF)
        {
            // negative number - extend the sign to 16th bit
            res |= 0xF000;
        }
        return (int16_t)res;
    }
}

#if 0


/**
 * @brief ADS1115_Init() - the following are the initialization steps needed for the ADS1115
 * 
 */
bool ADS1115_Init()
{

    ESP_LOGI(Tag,"%s called...",__func__);
    esp_err_t ret = generic_write_i2c_register_word(ADS1115_DEV_ADDR,ADS1115_CONFIG_REG, ADS1115_REG_RESET_VAL);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Error from generic_write_i2c_register_word = %s", esp_err_to_name(ret));
        return ret;
    }

    ADS1115_setVoltageRange_mV(ADS1115_RANGE_2048);


    //Review Adafruit_ADS1x15 settings! that should be golden
    ret = generic_write_i2c_register_word(ADS1115_DEV_ADDR,ADS1115_LO_THRESH_REG, ADS1115_LO_THRESH_VAL);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Error from generic_write_i2c_register_word = %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = generic_write_i2c_register_word(ADS1115_DEV_ADDR,ADS1115_HI_THRESH_REG, ADS1115_HI_THRESH_VAL);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Error from generic_write_i2c_register_word = %s", esp_err_to_name(ret));
        return ret;
    }
    
    return 1;
}

void ADS1115_clearAlert(void)
{
    int16_t rawResult;
    esp_err_t ret;
    ESP_LOGI(Tag, "ADS1115_clearAlert ");
    ret = generic_read_i2c_register_word(ADS1115_DEV_ADDR, ADS1115_CONFIG_REG, (uint16_t *)&rawResult);
    if(ret != ESP_OK)
    {
        ESP_LOGE(Tag, "Error from generic_read_i2c_register_word = %s", esp_err_to_name(ret));
        return;
    }
}

#endif