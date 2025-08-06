# ECGMonitor
The design of a portable 12-lead ECG monitoring device using a set cheap and off-the-shelf component boards. This design would incorporate principles of both thrift and resourcefulness. Useful skills to have in conditions of urgent care needs.

## Requirements
### HW required:
- ESP32 DevKit
- ADS1115       4 input channel 16-bit ADC I2C, running 860 samples per second. 
                    * Sources: https://www.adafruit.com/product/1085 , https://www.ti.com/product/ADS1115
                    * Datasheet: https://www.ti.com/lit/ds/symlink/ads1115.pdf?ts=1751944223631&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1115
- AD8232        Single-Lead, Heart Rate Monitor Front End. 
                    * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ad8232.pdf
- PCF8574       PCF8574 Remote 8-Bit I/O Expander for I2C Bus. This will handle the Leads-Off detection for up to AD8232 modules.
- MicroSDCardAdapter    ECG data will be stored on A 32GB SD card.
- LiPo 18650    rechargeable battery
- TP4056        charging board.
- DS3231        RTC module

### SW required:
- QRS Detector  measure beats at a minimum.
- Save ECG Data use sdspi & standard file io to write ECG data
- TBD. POST     Power-On Self Tests - to ensure the system is working & functioning correctly
## Development Environment Setup 

- Add Markdown Preview Enhanced VSCode extension to see a graphical view
- Important:
    - Keybindings
    * The cmd key for Windows is ctrl.
     Shortcuts	Functionality
    cmd-k v or ctrl-k v	Open preview to the Side
    cmd-shift-v or ctrl-shift-v	Open preview
    ctrl-shift-s	Sync preview / Sync source
    shift-enter	Run Code Chunk
    ctrl-shift-enter	Run all Code Chunks
    cmd-= or cmd-shift-=	Preview zoom in
    cmd-- or cmd-shift-_	Preview zoom out
    cmd-0	Preview reset zoom
    esc	Toggle sidebar TOC 


## Development plans

* Add real time clock to the project
    * Use DS3231 library (esp-idf-ds3231)

* Recycle Arduino based PCF8574 -- GPIO extender
    * This will be very useful for the "Leads Off" indicator
    * Fleshing Out LeadsOffDetector.c, converting from STM32 to ESP32 ESP-IDF

### Update 2925-08-05
* Too many issues that led to a fundamentally unworkable platform, in large part due to some rather vague technical documentation on the ESP-IDF platform. We are focused here on delivering some light on areas not suitably covered in the technical notes & manuals. Hopefully, this document could be a useful resource for those who want more light on some intricacies in the technology.

## Long term possibilities
The conception of this project was due in large part to a personal health crisis: While gardening in the backyard, I had been lifting and carrying around large bags of topsoil. I was definitely putting my body through too much of a workout, that ended up with a heart attack. Very stressful situation, and I am forever grateful to my next neighbor, Isaac, who rushed me to the hospital in time. I am exceedingly grateful to the Cardiac care I received by both Skokie Hospital and the Evanston Hospital, and I would be alive today without their expert care. This has been a humbling experience, and led me to realize how absolutely essential it is to have a robust and aggressive health care system in place for all. The idea for the portable ECG was to at least address my concerns to return to gardening - while I didn't want to sit around in fear that one false move could cause me to expire, I wanted someway to give me a way to know when a physical event has been signalled to indicate at a bare minimum that exertion must cease. It is very easy to get carried away with any physically involved activity, to the point where one can seriously jeopardize personal health. Being a senior citizen also makes me very aware of what older folks have to naturally endure as part of the ageing process. That has caused me to think about other possibilities that may prove to be beneficial for others.

## Development notes

* There have been issues with HW/SW interfacing for various devices that can be quite challenging to overcome
    * The MicroSD adapter can be tricky to work with. In the current instance. besides the issues with the strapping pin and the pull-down resistors (see below), there is an issue with the frequency for the SPI clock generation, which can result in failures to access the SD Card media, so investigation is needed. So far, I've the following piece of the current log messages:
    > I (342) TEST_ADS1115: app_main starting 
I (342) sdcard: Initializing SD card
I (342) sdcard: Using SPI peripheral
I (342) sdcard: Mounting filesystem
I (352) gpio: GPIO[13]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (392) sdspi_transaction: cmd=5, R1 response: command not supported
E (402) sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107
E (402) vfs_fat_sdmmc: sdmmc_card_init failed (0x107).
I (402) gpio: GPIO[13]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
E (412) sdcard: Failed to initialize the card (ESP_ERR_TIMEOUT). Make sure SD card lines have pull-up resistors in place.
E (422) TEST_ADS1115: Unable to initialize sdcard! = ESP_ERR_TIMEOUT
I (422) TEST_ADS1115: Fatal error
    * There are several references to this symptom on the Espressif developer forums that do have some things to try.
    * No effect when scaling the CLK frequency to 5000 (Hz?)

* New Log:
>W (322) i2c: This driver is an old driver, please migrate your application code to adapt driver/i2c_master.h
<span style="color: green;" >
I (332) main_task: Started on CPU0
I (342) main_task: Calling app_main()
I (342) CONTIN_READ: app_main starting
I (342) sdcard: Initializing SD card
I (342) sdcard: Using SPI peripheral
I (342) sdcard: Mounting filesystem
I (352) gpio: GPIO[13]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (392) sdspi_transaction: cmd=5, R1 response: command not supported
</span> <span style="color: red">
E (402) sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107
E (402) vfs_fat_sdmmc: sdmmc_card_init failed (0x107).
</span> <span style="color: green">
I (402) gpio: GPIO[13]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
</span> <span style="color: red">
E (412) sdcard: Failed to initialize the card (ESP_ERR_TIMEOUT). Make sure SD card lines have pull-up resistors in place.
E (422) CONTIN_READ: Unable to initialize sdcard! = ESP_ERR_TIMEOUT
</span> <span style="color:green">
I (422) CONTIN_READ: Fatal error
</span>


* We have to back to inspecting the pull-up resistors and the strapping pin items
Check all of the pins on the board.



*  ** DO NOT USE GPIO 2! This is a strapping pin. and using this can lead to problems with flashing: **

Incorrect Boot Mode Selection:

    ESP32 needs to be in download mode: To flash or connect to the ESP32, it needs to be in a specific download or bootloader mode.
    Strapping Pins: Certain I/O pins, known as strapping pins, control the boot mode. If these pins (especially GPIO0 and GPIO2) are not in the correct state during reset, the ESP32 might enter an unintended boot mode, such as HSPI_FLASH_BOOT (0xb).
        GPIO0: Should be held low during reset to enter the serial bootloader. If left floating, its internal pull-up resistor pulls it high, resulting in normal boot.
        GPIO2: Must be left unconnected or driven low to enter the serial bootloader. 

## References:

* Electrocardiograms basics
    * "Cardiac Electrophysiology Methods and Models", Iaizzo, Eggen, Iles - Springer 2nd Edition, ISBN 978-3-031-71066-7 : Part I contains an excellent overview of Cardiac Electrophysiology, which provides the much of the theoritical foundation for understanding the ECG.
    * 
* ADC for ECG capture
    * "ADS111x Ultra-Small, Low-Power, I2C-Compatible, 860SPS, 16-Bit ADCs with Internal Reference, Oscillator, and Programmable Comparator", datasheet for the ADS1115 4-channel I2C ADC chip which is used in this implemntation.
    * "Analog Front-End Design for ECG Systems Using Delta-Sigma ADCs", SBAA160A, Texas Instruments Application Report. 
    * "Analog-Digital Conversion Handbook", Engineering Staff of Analog Devices, Inc. Prentice-Hall, 1986 : considered by many to be the definitive guidebook on ADC.
* ECG Algorithms
    * "A Real-Time QRS Detection Algoritm", Jiapu Pan and Willis J. Tompkins, IEEE Transactions on Biomedical Engineering, Vol BME-32, March 1985. Principle metric of ECG analysis is the accurate detection and measurement of QRS complex found in the ECG signal capture. By measuring the "R-R" period in the ECG signal, we can derive beats-per-minute. 
* ECG File formats:




* Motivations: 
    * Heart Attack with emergency stent operation in April, 2024
    * "Barefoot Doctor's Manual" 
    * Microplastics and it's role in cardiovascular disease




