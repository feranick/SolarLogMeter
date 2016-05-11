SolarLogMeter
==============

Current Version: 3.7 - 20160509

Arduino sketch for a Log Meter device for solar PV measurements.

By Nicola Ferralis (feranick): feranick@hotmail.com

Features
========

- Simultaneous acquisition of up to 4 solar cell systems  
- Real time clock
- Extraction of relevant parameters based on IV curve
- Support for serial display or automatic acquisition.
- Support for saving data on SD card.
- Customizable configuration file.
- Support for temperature (via IC or thermistor) and barometric pressure
- Support for irradiance monitor
- Automatically computes and saves sun position
- Flexible activation/deactivation of used hardware.

Requirements
============
Required Library: Real Time Clock:
https://github.com/adafruit/RTClib

Requires Arduino MEGA 2560 or better (AVR) or Arduino DUE.
Initial support for Arduino Intel-based boards is included, but largely untested.

A data logging shield is recommended to save data in SD card, along with a custom configuration file. Recommended logging boards:

- Adafruit Logging shield


Configuration file
===================

System and operational configuration is read from the file SLG.CFG in the SD card. If the SD card is missing, default values are used.

- set if automatic acquisition (0) or manual through serial commands (1)
- number of cells (max: 4)
- Offset in current measurement (default: 0 mA)
- Max Voltage measured (default: 4.1 V)
- time in between IV scans (default: 12)
- unit time in between IV scans (0: min; 1: sec)
- Number of averages in voltage measurements (default: 80. Not used in v2)
- generic delay time (msecs) (default: 1000s ms)
- location latitude (default: 42.36)
- location longitude (default: -71.09)
- location timezone (default: -5.00)
- Daylight Saving Time (default: 1, yes).

Hardware configuration
=======================
 
 1. Controls a Microchip MCP4822 12-bit dual-voltage output DAC.
  - The MCP4822 is SPI-compatible. To command it, hold the CS pin low and send 16 bits,
  - 4 configuration bits
   - DAC selection: 0 = DACa, 1 = DACb
   - empty
   - gain selection: 1 = 1x, 0 = 2x with Vref = 2.048
   - output shutdown: 1 = Vout is available, 0 = Vout is not available R = 500 k-ohms
   - 12 data bits (0 - 4096)

 2. The circuit:
  - DACa ouput through buffer amp - to A0 and DUT+
  - DACb output - to op amp negative terminal thru 3.3k resistor
  - CS - to digital pin 10  (SS pin)
  - SDI - to digital pin 11 (MOSI pin)
  - SCK - to digital pin 13 (SCK pin)
  - Vout of current meter - to op amp positive terminal through 3.3k resistor
  - Vout of op amp (current measurement) - to A1
 
 3. Resistors for voltage dividers are labeled Rv1 and Rv2, while shunts for current 
 measurements are labeled Ri. They are arrays of floats, with number defined by the
 variable numCell. Therefore the number of values in the arrays needs to be properly
 adjusted according to numCell. 
 
 4. Temperature measurement: The current code supports only one thermistor. The values
 need to be properly set according to the right thermistor.   
 
 5. SD Card:
 If using the the Adafruit Logging shield with an Arduino Mega (Only the MEGA),
 in the file: ~arduino/libraries/SD/src/utility/Sd2Card.h
 
 a. change the line: 
    #define MEGA_SOFT_SPI 0
    to 
    #define MEGA_SOFT_SPI 1

 b. and comment:

    #define USE_SPI_LIB
    in ~arduino/libraries/SD/utility/Sd2Card.h
    and ~arduino/libraries/SD/utility/Sd2Card.c
 
 Do not change the other pins!
 Also make sure the definition below (SDshield) is correctly set for the type of SD shield used.
 
 6. Temperature measurements:
 
 There are two ways temperature can be measured, using:

 - thermistor (Vishay NTCLE100E3), pin 7 - (comment the definition of TBAR)
 - Bosch BMP085 Barometric sensor - via I2C bus (uncomment the definition of TBAR)
 
You must select at build time the type of sensor you want to use.
 
 7. Irradiance measurements:

 - Pins 2, 4, 5, 6, 7 for vertical irradiance.
 - Pins 3, 22, 23, 24, 25 for horizontal irradiance.
 
 8. Sun calculator:

 - Time is always given in Standard Daylight Time form (no daylight saving).


