SolarLogMeter v. 3.7 - 20160509
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

A data logging shield is recommended to save data in SD card, along with a custom configuration file.


Configuration file
===================

System and operational configuration is read from the file SLG.CFG in the SD card. If the SD card is missing, default values are used.

- set if automatic acquisition (0) or manual through serial commands (1)
- number of cells (max: 4)
- Offset in current measurement
- Max Voltage measured (stopV)
- time in between IV scans (msecs)
- unit time in between IV scans (0: min; 1: sec)
- number of averages
- generic delay time (msecs)
- location latitude
- location longitude
- location timezone
- Daylight Saving Time

