# HAB-FlightCode

This repository contains the Arduino-based flight controller code for a high-altitude balloon (HAB) project.

## Overview
The main sketch is `HAB_DeskTest_v1.0.ino. This integrates:
- GPS (u-blox MAX-M10S) via hardware serial with high-altitude flight mode  
- Environmental sensors: BMP180 (pressure) and DS18B20 (temperature)  
- RTTY telemetry transmission on pin D9  
- Logic-level MOSFET for altitude-based payload control  
- SD card logging to `DATA00.CSV`  
- External 5V indicator LEDs for GPS fix and RTTY activity  

This version represents the stable Desk Test Release (v1.0) — suitable for integration testing and further expansion.

## Hardware
- Arduino UNO R4 Minima  
- u-blox MAX-M10S GPS (Uptronics breakout)  
- BMP180 pressure sensor  
- DS18B20 temperature sensor  
- SD card module (SPI)  
- Logic-level MOSFET for altitude-triggered payload control  
- Pololu S9V11F5 5V Step-Up/Down Voltage Regulator  
- External indicator LEDs:  
  - GPS Fix LED: blinks until GPS lock, solid ON when locked  
  - RTTY LED: flashes with each RTTY transmission  

##  Telemetry
- RTTY: 50 baud, 7N2 (dl-fldigi compatible)  
- Example transmission string (UKHAS format with checksum):
 -   $$HAB1,31,-41,54.000000,-1.000000,8,MOS_ON*5E

