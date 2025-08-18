# HAB-FlightCode

Arduino-based flight software for a High Altitude Balloon (HAB) project.  
The system gathers sensor data, transmits telemetry over RTTY, and logs information to an SD card for post-flight analysis.  

## ✈️ Features
- GPS telemetry with u-blox MAX-M10S in high-altitude flight mode.  
- RTTY transmission (dl-fldigi compatible, UKHAS standard).  
- Environmental data logging:
  - BMP180 (pressure/altitude/temperature).  
  - DS18B20 (external temperature).  
- MOSFET control for onboard systems (auto ON/OFF above threshold altitude).  
- LED indicators for GPS fix and RTTY transmission.  
- SD card logging (independent from RTTY).  

## 🛠️ Hardware
- Arduino Uno R4 Minima  
- u-blox MAX-M10S GPS (via Uptronics breakout board)
- Radiometrix NTX2B-FA 434.300 Mz  
- BMP180 pressure sensor  
- DS18B20 external temperature sensor  
- SD card module (logging to `DATA00.CSV`)  
- MOSFET (altitude-triggered payload control)  
- External indicator LEDs:
  - GPS fix LED (blinks until fix, solid when locked).  
  - RTTY LED (flashes with each RTTY transmission).  

## 📡 Telemetry
- RTTY: 50 baud, 7N2 (dl-fldigi compatible).  
- Example transmission string (UKHAS format with checksum):
 -   $$HAB1,31,-41,54.000000,-1.000000,8,MOS_ON*5E

