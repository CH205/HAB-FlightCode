/*
  Acknowledgment: This project was developed with substantial assistance from ChatGPT,
  a large language model developed by OpenAI. ChatGPT contributed to the coding
  structure, debugging, and integration of multiple hardware components into a cohesive
  high-altitude balloon flight system.
*/

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <stdarg.h>

// Pins
#define RTTY_PIN       9
#define SOLENOID_PIN   6
#define SD_CS         10
#define ONE_WIRE_BUS   4
#define LED_PIN       13
#define GPS_RX         8
#define GPS_TX         7

// Globals
Adafruit_BMP085 bmp;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
File dataFile;
char filename[] = "DATA00.CSV";
unsigned long startTime, lastSensor, lastBlink, lastRTTY;
bool solenoidFired = false;
bool solenoidReset = false;
const float QNH = 1013.25;

// Timing
const unsigned long sensorInterval = 1000;
const unsigned long blinkInterval  = 500;
const unsigned long rttyInterval   = 1000;
const unsigned long rttyOnTime     = 50;

// Helper for printf-style Serial output
void serialPrintf(const char *fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

void logEvent(const String &evt) {
  unsigned long elapsed = (millis() - startTime) / 1000;
  Serial.println("EVENT: " + evt);
  dataFile.println(String(elapsed) + "," + evt);
  dataFile.flush();
}

void readAndLog() {
  float temp     = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;
  float altitude = 44330.0 * (1.0 - pow(pressure/QNH, 0.1903));
  sensors.requestTemperatures();
  float extTemp = sensors.getTempCByIndex(0);
  if (extTemp == DEVICE_DISCONNECTED_C) extTemp = -999.0;

  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  String lat = gps.location.isValid() ? String(gps.location.lat(),6) : "0.000000";
  String lon = gps.location.isValid() ? String(gps.location.lng(),6) : "0.000000";
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  unsigned long elapsed = (millis() - startTime) / 1000;

  // Print to Serial
  serialPrintf("Time: %lus | Temp: %.2f°C | Press: %.2fhPa | Alt: %.2fm | Lat: %s Lon: %s | Sats: %d | ExtTemp: %.2f°C\n",
    elapsed, temp, pressure, altitude, lat.c_str(), lon.c_str(), sats, extTemp);

  // Solenoid control
  if (altitude >= 38.5 && !solenoidFired) {
    solenoidFired = true;
    digitalWrite(SOLENOID_PIN, HIGH);
    logEvent("SOLENOID ON");
  }
  if (altitude < 38.0 && solenoidFired && !solenoidReset) {
    solenoidReset = true;
    digitalWrite(SOLENOID_PIN, LOW);
    logEvent("SOLENOID OFF");
  }

  // Data CSV
  String csv = String(elapsed) + "," + String(temp,2) + "," +
               String(pressure,2) + "," + String(altitude,2) + "," +
               lat + "," + lon + "," + String(sats) + "," + String(extTemp,2);
  if (solenoidFired && !solenoidReset) csv += ",SOLENOID ON";
  else if (solenoidReset) csv += ",SOLENOID OFF";
  dataFile.println(csv);
  dataFile.flush();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RTTY_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(57600);  // Lower baud helps avoid gibberish :contentReference[oaicite:5]{index=5}
  gpsSerial.begin(9600);
  delay(500);
  Serial.println(F("*** SETUP START"));

  bmp.begin();
  sensors.begin();
  SD.begin(SD_CS);
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      break;
    }
  }
  dataFile.println("Time,Temp,Pressure,Altitude,Lat,Lon,Sats,ExtTemp,Event");
  dataFile.flush();

  startTime = lastSensor = lastBlink = lastRTTY = millis();
  Serial.println(F("*** SETUP COMPLETE"));
}

void loop() {
  unsigned long now = millis();

  if (now - lastBlink >= blinkInterval) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = now;
  }

  if (now - lastRTTY >= rttyInterval) {
    digitalWrite(RTTY_PIN, HIGH);
    lastRTTY = now;
    logEvent("RTTY pulse");
  }
  if (digitalRead(RTTY_PIN) && now - lastRTTY >= rttyOnTime) {
    digitalWrite(RTTY_PIN, LOW);
  }

  if (now - lastSensor >= sensorInterval) {
    lastSensor = now;
    readAndLog();
  }
}
