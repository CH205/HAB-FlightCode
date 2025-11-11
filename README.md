# HAB-FlightCode

**High Altitude Balloon Flight Controller**

This Arduino-based project is designed to control and monitor a high-altitude balloon payload.  
It integrates sensor logging, RTTY telemetry transmission, GPS tracking, and automated solenoid control — all with robust error handling and visual status indicators.

---

## ✈️ Overview

The flight controller coordinates the following systems:

- **GPS (u-blox MAX-M10S)** for position, altitude, and time data.  
- **BMP180** barometric pressure sensor for altitude estimation.  
- **DS18B20** digital thermometer for temperature readings.  
- **SD card logging** for complete data recording.  
- **RTTY telemetry transmitter** for real-time downlink to ground station.  
- **MOSFET -  for altitude-triggered payload.  
- **LED indicators** for:
  - GPS fix status (blinking until fix, solid when locked).  
  - RTTY transmission activity (flashes on each pulse).  

---

## ⚙️ Hardware

- **Microcontroller:** Arduino Uno R4 Minima  
- **GPS Module:** u-blox MAX-M10S (Uptronics breakout)  
- **Sensors:** BMP180, DS18B20  
- **Storage:** SD card module  
- **Indicators:** 5V LEDs on D3 (RTTY) and D5 (GPS fix)
- Pololu S9V11F5 5V Step-Up/Down Voltage Regulator  
- MOSFET on configured output pin  
- **RTTY Output:** Pin D9  

---

## 🛰️ Features

- Full telemetry transmission decoded in **dl-fldigi**.  
- SD logging independent of RTTY operation.  
- Altitude-based MOSFET-activated device with event logging.  
- Modular and robust code structure for flight reliability.  
- Easily expandable for future enhancements.  

---

## 🧾 Version History

### v1.2 — Current Release (November 2025)
- Increased telemetry buffer size to 200 characters for greater message stability.
- Maintains full functionality of v1.1, including:
  - Reliable GPS, BMP180, and DS18B20 sensor data logging.
  - SD card logging to `DATA00.CSV`.
  - RTTY transmission with indicator LED (D3).
  - GPS fix indicator LED (D5).
  - Device control above-defined-altitude threshold.
- Verified transmission clarity via dl-fldigi.

### v1.1 — Stable Foundation (August 2025)
- Integrated full flight control logic with environmental sensors, GPS, SD logging, RTTY, and solenoid.
- First publicly stable version, serving as the baseline for future development.

---

## 🤝 Acknowledgments

Developed with support and guidance from **ChatGPT (OpenAI)** —  
assisting in design, debugging, and documentation.

---

## 📜 License

This project is open-source and free to use for educational and non-commercial applications.  
Attribution is appreciated if you use or adapt the code.
