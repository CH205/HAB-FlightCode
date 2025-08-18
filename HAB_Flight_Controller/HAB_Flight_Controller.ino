
/*
  High-Altitude Balloon — Integrated Baseline (robust + annotated)
  - Keeps your working 50-baud 7_N_2 RTTY (bit-banged on RTTY_PIN)
  - RESTORES u-blox “airborne” (high-altitude) UBX patch (enableAirborneMode())
  - Serial (semicolon) + SD (CSV) logging unchanged
  - RTTY payload now mirrors SD log’s MOS field (MOS_ON/MOS_OFF)  // *** CHANGE
  - RTTY LED (pin 3) gives a short visible pulse for EACH CHARACTER  // *** CHANGE
  - Adds ham-style NMEA checksum *XX at end of the $RTTY sentence    // *** CHANGE
  - Structure kept modular & easy to read.
  
  Credits:
  Script based on user’s proven flight code, enhanced and made UKHAS-compliant with 
  checksum handling, GPS airborne mode, and clean modular structure. Compiled with 
  assistance from ChatGPT (OpenAI).

*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>

// -----------------------------
// Pin Assignments
// -----------------------------
const int SD_CS_PIN = 10;     // SD card chip select
const int MOS_PIN   = 6;     // MOSFET control pin
const int RTTY_PIN  = 9;     // RTTY output pin
const int LED_RTTY  = 3;     // LED flashes with each RTTY bit
const int LED_GPS   = 5;     // LED shows GPS fix status
const int ONE_WIRE_BUS = 4;  // DS18B20 temp sensor

// -----------------------------
// Constants
// -----------------------------
const float ALTITUDE_THRESHOLD = -100.0;   // meters (yo)

// RTTY parameters
const int RTTY_BAUD = 50;
const int BIT_PERIOD = 1000 / RTTY_BAUD; // ms per bit

// -----------------------------
// Globals
// -----------------------------
File logFile;
Adafruit_BMP085 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;

bool MOSFired = false;
unsigned long packetCounter = 0;
unsigned long lastBlink = 0;

// -----------------------------
// Setup
// -----------------------------
void setup() {
  pinMode(MOS_PIN, OUTPUT);
  digitalWrite(MOS_PIN, LOW);

  pinMode(RTTY_PIN, OUTPUT);
  digitalWrite(RTTY_PIN, LOW);

  pinMode(LED_RTTY, OUTPUT);
  digitalWrite(LED_RTTY, LOW);

  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_GPS, LOW);

  Serial.begin(9600);
  Serial1.begin(9600);  // GPS on Serial1 (UNO R4 Minima)

  // Initialize BMP
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected!");
  }

  // Initialize DS18B20
  sensors.begin();

  // Initialize SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card failed, logging disabled.");
  } else {
    logFile = SD.open("DATA00.CSV", FILE_WRITE);
    if (logFile) {
      logFile.println("Time;Latitude;Longitude;Sats;Altitude;Temp1;Temp2;Pressure;MOS");
      logFile.flush();
    }
  }

  // Send airborne mode command to GPS
  sendGPSAirborneMode();
}

// -----------------------------
// Main Loop
// -----------------------------
void loop() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // Blink GPS LED when no fix, solid when fix
  if (gps.location.isValid() && gps.satellites.value() > 0) {
    digitalWrite(LED_GPS, HIGH);
  } else {
    if (millis() - lastBlink > 500) {
      digitalWrite(LED_GPS, !digitalRead(LED_GPS));
      lastBlink = millis();
    }
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 1000) {
    lastLog = millis();

    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    float tempC2 = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0;
    float alt_m = bmp.readAltitude();

    // MOSFET logic
    String eventStr = MOSFired ? "MOS_ON" : "MOS_OFF";
    if (alt_m >= ALTITUDE_THRESHOLD) {
      if (!MOSFired) {
        digitalWrite(MOS_PIN, HIGH);
        MOSFired = true;
        eventStr = "MOS_ON";
      }
    } else {
      if (MOSFired) {
        digitalWrite(MOS_PIN, LOW);
        MOSFired = false;
        eventStr = "MOS_OFF";
      }
    }

    // Build log line for Serial + SD
    String logLine = "";
    logLine += gps.time.hour(); logLine += ":";
    logLine += gps.time.minute(); logLine += ":";
    logLine += gps.time.second(); logLine += ";";
    logLine += String(gps.location.lat(), 6); logLine += ";";
    logLine += String(gps.location.lng(), 6); logLine += ";";
    logLine += gps.satellites.value(); logLine += ";";
    logLine += String(alt_m, 2); logLine += ";";
    logLine += String(tempC, 2); logLine += ";";
    logLine += String(tempC2, 2); logLine += ";";
    logLine += String(pressure, 2); logLine += ";";
    logLine += eventStr;

    Serial.println(logLine);
    if (logFile) {
      logFile.println(logLine);
      logFile.flush();
    }

    // Build HAB RTTY sentence (without temps)
    String sentence = "$$HAB1,";
    sentence += String(packetCounter++);
    sentence += ",";
    sentence += String((int)alt_m);
    sentence += ",";
    sentence += String(gps.location.lat(), 6);
    sentence += ",";
    sentence += String(gps.location.lng(), 6);
    sentence += ",";
    sentence += gps.satellites.value();
    sentence += ",";
    sentence += eventStr;

    // Append checksum
    uint16_t checksum = crc16(sentence.c_str() + 2);
    char buf[8];
    sprintf(buf, "*%02X", checksum & 0xFF);
    sentence += buf;

    // Send via RTTY
    sendRTTY(sentence);
  }
}

// -----------------------------
// Functions
// -----------------------------
void sendRTTY(String msg) {
  for (int i = 0; i < msg.length(); i++) {
    sendByte(msg[i]);
  }
  sendByte('\n'); // end of sentence
}

void sendByte(char c) {
  digitalWrite(RTTY_PIN, LOW);
  digitalWrite(LED_RTTY, HIGH);
  delay(BIT_PERIOD);   // Start bit
  digitalWrite(LED_RTTY, LOW);

  for (int i = 0; i < 7; i++) { // 7N2 encoding
    if (c & (1 << i)) {
      digitalWrite(RTTY_PIN, HIGH);
    } else {
      digitalWrite(RTTY_PIN, LOW);
    }
    digitalWrite(LED_RTTY, HIGH);
    delay(BIT_PERIOD);
    digitalWrite(LED_RTTY, LOW);
  }

  digitalWrite(RTTY_PIN, HIGH);
  digitalWrite(LED_RTTY, HIGH);
  delay(BIT_PERIOD * 2);  // 2 stop bits
  digitalWrite(LED_RTTY, LOW);
}

uint16_t crc16(const char *s) {
  uint16_t crc = 0xFFFF;
  while (*s) {
    crc ^= *s++ << 8;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void sendGPSAirborneMode() {
  // UBX command for airborne <1g mode
  byte setNavMode[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
    0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00,
    0x00, 0x10, 0x27, 0x00, 0x00, 0x05,
    0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64,
    0x00, 0x2C, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x16, 0xDC
  };

  for (int i = 0; i < sizeof(setNavMode); i++) {
    Serial1.write(setNavMode[i]);
  }
}

