
/* ============================================================
   HAB_DeskTest_v1.0.ino
   ------------------------------------------------------------
   High Altitude Balloon – Desk Test Build
   TRANSMITS: RTTY (7N2, 50 baud, stable timing)
   LOGS: SD card (auto recovery)
   READS: GPS (airborne mode patch), BMP180, DS18B20
   CONTROLS: MOSFET trigger (altitude threshold)
   INDICATORS: GPS fix LED + RTTY LED
   FEATURES:
   - GPS watchdog to suppress stale/ghost data
   - Auto SD reinit
   - Compact serial status output
   - 100 m trigger threshold (for bench testing)
   ------------------------------------------------------------
   This code was developed in collaboration with ChatGPT (GPT-5, OpenAI)
   ============================================================ */

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <inttypes.h>

// ==================== PIN ASSIGNMENTS ====================
#define RTTY_PIN      9
#define RTTY_LED_PIN  3
#define GPS_LED_PIN   5
#define MOS_PIN       6
#define ONE_WIRE_BUS  4
#define SD_CS         10

// ==================== OBJECTS ====================
Adafruit_BMP085 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
SoftwareSerial gpsSerial(8, 7);  // RX = 8, TX = 7
TinyGPSPlus gps;

// ==================== GLOBALS ====================
unsigned long sentenceCounter = 0;
const float ALT_THRESHOLD = 100.0;  // altitude threshold for MOSFET
bool sdAvailable = false;

// ==================== GPS AIRBORNE MODE PATCH ====================
void sendGPSAirborneMode() {
  uint8_t setAirborne[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
    0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x16, 0xDC
  };
  for (unsigned int i = 0; i < sizeof(setAirborne); i++) {
    gpsSerial.write(setAirborne[i]);
  }
}

// ==================== RTTY FUNCTIONS ====================
#define BIT_PERIOD 20000UL  // 50 baud

void rtty_txbit(bool bit) {
  digitalWrite(RTTY_PIN, bit ? HIGH : LOW);
  digitalWrite(RTTY_LED_PIN, bit ? HIGH : LOW);
  delayMicroseconds(BIT_PERIOD);
}

void rtty_txbyte(char c) {
  rtty_txbit(0);  // start bit
  for (int i = 0; i < 7; i++) {
    rtty_txbit((c >> i) & 0x01);
  }
  rtty_txbit(1);  // stop bits
  rtty_txbit(1);
}

void rtty_txstring(const char *s) {
  while (*s) rtty_txbyte(*s++);
}

// ==================== SD LOGGING ====================
void logToSD(const char *sentence) {
  if (!sdAvailable) {
    if (SD.begin(SD_CS)) {
      Serial.println("SD reinit successful.");
      sdAvailable = true;
    } else {
      Serial.println("SD unavailable.");
      return;
    }
  }

  File logfile = SD.open("DATA00.CSV", FILE_WRITE);
  if (logfile) {
    logfile.print(sentence);
    logfile.close();
  } else {
    Serial.println("ERROR: SD write failed.");
    sdAvailable = false;
  }
}

// ==================== SETUP ====================
void setup() {
  pinMode(RTTY_PIN, OUTPUT);
  pinMode(RTTY_LED_PIN, OUTPUT);
  pinMode(GPS_LED_PIN, OUTPUT);
  pinMode(MOS_PIN, OUTPUT);

  Serial.begin(9600);
  gpsSerial.begin(9600);

  if (!bmp.begin()) Serial.println("BMP180 not found!");
  sensors.begin();

  if (SD.begin(SD_CS)) {
    Serial.println("SD init success.");
    sdAvailable = true;
    if (!SD.exists("DATA00.CSV")) {
      File logfile = SD.open("DATA00.CSV", FILE_WRITE);
      if (logfile) {
        logfile.println("ID,Count,Alt,Temp1,Temp2,Pressure,Lat,Lon,Sats,MOSFET,Checksum");
        logfile.close();
      }
    }
  } else {
    Serial.println("SD init failed!");
  }

  sendGPSAirborneMode();
  Serial.println("Setup complete.");
}

// ==================== MAIN LOOP ====================
void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  static unsigned long lastFixTime = 0;
  static bool gpsLost = false;
  bool gpsFix = gps.location.isValid();

  if (gpsFix) {
    lastFixTime = millis();
    gpsLost = false;
  } else if (millis() - lastFixTime > 10000) {
    if (!gpsLost) Serial.println("GPS data invalidated (no signal >10s)");
    gpsLost = true;
  }

  float tempC1 = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;
  sensors.requestTemperatures();
  float tempC2 = sensors.getTempCByIndex(0);
  float alt_m = bmp.readAltitude();

  bool mosfetOn = false;
  if (alt_m > ALT_THRESHOLD) {
    digitalWrite(MOS_PIN, HIGH);
    mosfetOn = true;
  } else {
    digitalWrite(MOS_PIN, LOW);
  }

  // Build telemetry sentence
  char sentence[120];
  snprintf(sentence, sizeof(sentence),
           "$$HAB1,%lu,%.1f,%.1f,%.1f,%.1f,%.6f,%.6f,%d,%s",
           sentenceCounter,
           alt_m,
           tempC1,
           tempC2,
           pressure,
           gpsLost ? 0.0 : gps.location.lat(),
           gpsLost ? 0.0 : gps.location.lng(),
           gpsLost ? 0 : gps.satellites.value(),
           mosfetOn ? "MosON" : "MosOFF");

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < strlen(sentence); i++) checksum ^= sentence[i];

  char fullSentence[120];
  snprintf(fullSentence, sizeof(fullSentence), "%s*%02X\r\n", sentence, checksum);

  Serial.println(fullSentence);
  logToSD(fullSentence);
  rtty_txstring(fullSentence);

  // LED logic
  if (gps.location.isValid()) {
    digitalWrite(GPS_LED_PIN, HIGH);
  } else if (gps.charsProcessed() > 0) {
    digitalWrite(GPS_LED_PIN, (millis() % 1000 < 50));
  } else {
    digitalWrite(GPS_LED_PIN, LOW);
  }

  // Serial status summary
  Serial.print("Count: "); Serial.print(sentenceCounter);
  Serial.print(" | GPS: "); Serial.print(gpsLost ? "LOST" : "OK");
  Serial.print(" | SD: "); Serial.print(sdAvailable ? "OK" : "FAIL");
  Serial.print(" | MOSFET: "); Serial.println(mosfetOn ? "ON" : "OFF");

  sentenceCounter++;
  delay(1000);
}
