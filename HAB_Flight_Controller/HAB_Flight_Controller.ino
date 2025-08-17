/*
  High-Altitude Balloon — Integrated Baseline (robust + annotated)
  - Keeps your working 50-baud 7N2 RTTY (bit-banged on RTTY_PIN)
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
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <math.h>

// ---------------- Pin configuration (unchanged) ----------------
#define MOS_PIN        6
#define ONE_WIRE_BUS   4
#define RTTY_PIN       9
#define RTTY_LED_PIN   3
#define LED_PIN        13
#define SD_CS          10
#define GPS_LED_PIN    5

// ---------------- Settings (unchanged) ----------------
const float QNH = 1013.25;                   // hPa
const float ALTITUDE_THRESHOLD = -100.0;      // meters (your test value)
const unsigned long SENSOR_INTERVAL = 1000;  // ms
const unsigned long RTTY_INTERVAL   = 2000;  // ms
const unsigned long BLINK_INTERVAL  = 500;   // ms

// RTTY timing (50 baud, 7N2)
#define RTTY_BAUD 50
static const uint16_t RTTY_BIT_US = 1000000UL / RTTY_BAUD;

// Optional: ham-style NMEA checksum (set 0 to disable) // *** CHANGE
#define RTTY_USE_NMEA_CHECKSUM 1

// ---------------- Hardware objects (unchanged) ----------------
Adafruit_BMP085 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
HardwareSerial &gpsSerial = Serial1;

File dataFile;
char filename[] = "DATA00.CSV";

// ---------------- State ----------------
unsigned long startTime = 0;
unsigned long lastSensor = 0;
unsigned long lastRTTY   = 0;
unsigned long lastBlink  = 0;
bool MOSFired = false;

// ---------------- Forward declarations ----------------
void enableAirborneMode();
void sendRTTYByte(char c);
void sendRTTYString(const char *s);
void transmitRTTY(const String &payload);
String addNMEAChecksum(const String &sentence); // *** CHANGE

// (left declared for future modularization; not used here)
String buildSerialLine(unsigned long, const String&, const String&, int, float, float, float, float, const String&);
String buildCSVLine(unsigned long, const String&, const String&, int, float, float, float, float, const String&);

// ---------------- Setup ----------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RTTY_PIN, OUTPUT);
  pinMode(RTTY_LED_PIN, OUTPUT);
  pinMode(MOS_PIN, OUTPUT);
  pinMode(GPS_LED_PIN, OUTPUT);

  digitalWrite(MOS_PIN, LOW);
  digitalWrite(RTTY_PIN, HIGH);     // RTTY idle = MARK = HIGH
  digitalWrite(RTTY_LED_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(57600);
  gpsSerial.begin(9600);
  delay(300);

  Serial.println(F("*** SETUP START"));

  // ---- Flight mode patch for u-blox GPS (AIRBORNE) ---- // RESTORED
  // Required for HAB flights: increases altitude/speed limits.
  enableAirborneMode();

  // Sensors
  if (!bmp.begin()) {
    Serial.println(F("BMP init failed"));
  }
  sensors.begin();

  // SD init + header (CSV)
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD init failed!"));
  } else {
    Serial.println(F("SD initialized."));
    for (uint8_t i = 0; i < 100; i++) {
      filename[4] = '0' + i / 10;
      filename[5] = '0' + i % 10;
      if (!SD.exists(filename)) {
        dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
          dataFile.println(F("timestamp,lat,lon,sats,alt_m,temp_bmp,ext_temp,pressure_hPa,Event"));
          dataFile.close();
        }
        break;
      }
    }
  }

  startTime = lastSensor = lastBlink = lastRTTY = millis();
  Serial.println(F("*** SETUP COMPLETE"));
}

// ---------------- Main loop ----------------
void loop() {
  // Feed GPS parser
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  unsigned long now = millis();

  // Blink onboard LED (heartbeat)
  if (now - lastBlink >= BLINK_INTERVAL) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = now;
  }

  // ---- RTTY transmit every RTTY_INTERVAL ----
  if (now - lastRTTY >= RTTY_INTERVAL) {
    lastRTTY = now;

    // Build MOS state string here so RTTY mirrors SD log // *** CHANGE
    String mosStr = MOSFired ? "MOS_ON" : "MOS_OFF";

    // Prepare concise RTTY payload (using real sensor/GPS data)
    float pressure_hPa = bmp.readPressure() / 100.0;
    float alt_m = 44330.0 * (1.0 - pow(pressure_hPa / QNH, 0.1903));
    sensors.requestTemperatures();
    float extTemp = sensors.getTempCByIndex(0);
    if (extTemp == DEVICE_DISCONNECTED_C) extTemp = -999.0;
    String latStr = gps.location.isValid() ? String(gps.location.lat(), 6) : String(0.0, 6);
    String lonStr = gps.location.isValid() ? String(gps.location.lng(), 6) : String(0.0, 6);
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    unsigned long elapsed = (now - startTime) / 1000;

    // Core sentence (no checksum yet)
    char core[160];
    snprintf(core, sizeof(core),
             "$RTTY,%lu,%.1fm,%s,%s,%d,%s",   // *** CHANGE: added ,MOS_xxx at end
             elapsed, alt_m, latStr.c_str(), lonStr.c_str(), sats, mosStr.c_str());

    // Optional NMEA-style checksum *XX               // *** CHANGE
    String toSend =
#if RTTY_USE_NMEA_CHECKSUM
      addNMEAChecksum(String(core));
#else
      String(core);
#endif

    transmitRTTY(toSend);  // (sends CRLF automatically)
    // Intentionally do NOT Serial.println(toSend) or log this to SD.
  }

  // ---- Periodic sensor read & unified logging ----
  if (now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;

    // Read sensors and GPS values
    sensors.requestTemperatures();
    float extTemp = sensors.getTempCByIndex(0);
    if (extTemp == DEVICE_DISCONNECTED_C) extTemp = -999.0;
    float temp_bmp = bmp.readTemperature();
    float pressure_hPa = bmp.readPressure() / 100.0;
    float alt_m = 44330.0 * (1.0 - pow(pressure_hPa / QNH, 0.1903));
    String latStr = gps.location.isValid() ? String(gps.location.lat(), 6) : String(0.0, 6);
    String lonStr = gps.location.isValid() ? String(gps.location.lng(), 6) : String(0.0, 6);
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // Timestamp: GPS time if valid (HH:MM:SS), else elapsed seconds
    unsigned long timestampOrElapsed;
    bool useGPSTime = false;
    if (gps.time.isValid()) {
      int hh = gps.time.hour();
      int mm = gps.time.minute();
      int ss = gps.time.second();
      if (hh >= 0 && mm >= 0 && ss >= 0) {
        timestampOrElapsed = hh * 3600UL + mm * 60UL + ss;
        useGPSTime = true;
      } else {
        timestampOrElapsed = (now - startTime) / 1000;
      }
    } else {
      timestampOrElapsed = (now - startTime) / 1000;
    }

    // MOSFET logic based on pressure-derived altitude (unchanged)
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

    // GPS LED behaviour (unchanged)
    if (gps.location.isValid()) {
      digitalWrite(GPS_LED_PIN, HIGH);
    } else {
      digitalWrite(GPS_LED_PIN, (millis() / 500) % 2);
    }

    // Build semicolon Serial line (unchanged)
    String serialTime;
    if (useGPSTime && timestampOrElapsed < 86400UL) {
      int hh = timestampOrElapsed / 3600;
      int mm = (timestampOrElapsed % 3600) / 60;
      int ss = timestampOrElapsed % 60;
      char tbuf[16];
      snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", hh, mm, ss);
      serialTime = String(tbuf);
    } else {
      serialTime = String(timestampOrElapsed);
    }

    String serialLine = serialTime + ";" + latStr + ";" + lonStr + ";" + String(sats) + ";" +
                        String(alt_m, 2) + ";" + String(temp_bmp, 2) + ";" + String(extTemp, 2) + ";" +
                        String(pressure_hPa, 2) + ";" + eventStr;

    Serial.println(serialLine);

    // Build CSV for SD (unchanged)
    String csvTime = serialTime;
    String csvLine = csvTime + "," + latStr + "," + lonStr + "," + String(sats) + "," +
                     String(alt_m, 2) + "," + String(temp_bmp, 2) + "," + String(extTemp, 2) + "," +
                     String(pressure_hPa, 2) + "," + eventStr;

    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println(csvLine);
      dataFile.close();
    }
  }
}

// ---------------- u-blox Flight-mode / UBX patch (RESTORED) ----------------
void enableAirborneMode() {
  // UBX-CFG-NAV5 (set dynamic model to Airborne)
  const uint8_t setAirborne[] = {
    0xB5, 0x62, 0x06, 0x24,
    0x24, 0x00,
    0x01, 0x06,
    0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x10, 0x27, 0x00, 0x00,
    0x05, 0x00,
    0xFA, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
  };
  uint8_t ck_a = 0, ck_b = 0;
  for (uint8_t i = 2; i < sizeof(setAirborne); i++) {
    ck_a += setAirborne[i];
    ck_b += ck_a;
  }
  for (uint8_t i = 0; i < sizeof(setAirborne); i++) gpsSerial.write(setAirborne[i]);
  gpsSerial.write(ck_a);
  gpsSerial.write(ck_b);
  delay(200);
}

// ---------------- RTTY helpers ----------------

// *** CHANGE: Short LED "blip" at the start of EACH CHARACTER without altering bit timing.
void sendRTTYByte(char c) {
  // Start bit (0 = SPACE)
  digitalWrite(RTTY_PIN, LOW);
  // LED pulse: half a bit ON, half a bit OFF, total still 1 bit period
  digitalWrite(RTTY_LED_PIN, HIGH);
  delayMicroseconds(RTTY_BIT_US / 2);
  digitalWrite(RTTY_LED_PIN, LOW);
  delayMicroseconds(RTTY_BIT_US - (RTTY_BIT_US / 2));

  // 7 data bits, LSB first
  for (int i = 0; i < 7; i++) {
    digitalWrite(RTTY_PIN, (c >> i) & 0x01);
    delayMicroseconds(RTTY_BIT_US);
  }

  // 2 stop bits (1 = MARK)
  digitalWrite(RTTY_PIN, HIGH);
  delayMicroseconds(RTTY_BIT_US);
  delayMicroseconds(RTTY_BIT_US);
}

void sendRTTYString(const char *s) {
  while (*s) sendRTTYByte(*s++);
}

void transmitRTTY(const String &payload) {
  // Send the sentence then CRLF (keeps dl-fldigi lines clean) // *** CHANGE
  sendRTTYString(payload.c_str());
  sendRTTYByte('\r');
  sendRTTYByte('\n');
}

// ---------------- Ham-style checksum (NMEA XOR) ----------------
// Computes 8-bit XOR of characters BETWEEN the '$' and the '*' (or end).
// Returns: "<sentence>*HH" where HH is uppercase hex.               // *** CHANGE
String addNMEAChecksum(const String &sentence) {
  int start = sentence.indexOf('$');
  if (start < 0) return sentence;  // no '$' found; return unchanged

  uint8_t x = 0;
  for (int i = start + 1; i < sentence.length(); i++) {
    char c = sentence.charAt(i);
    if (c == '*') break;
    x ^= (uint8_t)c;
  }
  char hex[4];
  snprintf(hex, sizeof(hex), "%02X", x);
  return sentence + "*" + String(hex);
}

// ---------------- (placeholders for future modularization) ----------------
String buildSerialLine(unsigned long, const String&, const String&, int, float, float, float, float, const String&) { return String(); }
String buildCSVLine(unsigned long, const String&, const String&, int, float, float, float, float, const String&) { return String(); }
