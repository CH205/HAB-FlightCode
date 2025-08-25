/*feat: establish robust failsafe Nano flight code (baseline release).

This commit marks the first robust baseline of the Nano high-altitude balloon flight code. It
has been ground-tested for stability and decoding reliability. Key features include:

- GPS (u-blox MAX-M10S) running on hardware serial with airborne flight mode enabled
- Clean, decodable RTTY telemetry with LED indicator pulse
- GPS fix LED (blinking until fix, solid when fixed)
- MOSFET altitude-triggered logic with logging of ON/OFF events
- Reliable SD card logging to DATA00.CSV (with simplified checksum handling)
- Telemetry continues even without SD card present (failsafe)

Acknowledgement: Script structure and integration support provided by ChatGPT (OpenAI).*/


#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>

// --- Pin Assignments ---
#define MOS_PIN 6
#define RTTY_PIN 9
#define GPS_LED 5
#define RTTY_LED 3
#define ONE_WIRE_BUS 4
#define SD_CS 10

// --- Constants ---
const float ALTITUDE_THRESHOLD = -100.0;  // meters (change on the day)
const int RTTY_BAUD = 50;                 // RTTY baud rate

// --- Globals ---
Adafruit_BMP085 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
File logfile;

bool MOSFired = false;
unsigned long sentenceCounter = 0;

// --- Forward declarations ---
void rtty_txstring(const char *string);
void rtty_txbyte(char c);
void rtty_txbit(int bit);
void sendGPSAirborneMode();

void setup() {
  pinMode(MOS_PIN, OUTPUT);
  digitalWrite(MOS_PIN, LOW);

  pinMode(RTTY_PIN, OUTPUT);
  digitalWrite(RTTY_PIN, LOW);

  pinMode(GPS_LED, OUTPUT);
  pinMode(RTTY_LED, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(9600);
  delay(250);
  sendGPSAirborneMode();  // enable airborne <1g model

  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
  }

  sensors.begin();

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed or not present.");
  } else {
    logfile = SD.open("DATA00.CSV", FILE_WRITE);
    if (logfile) {
      logfile.println("Sentence,Count,Alt(m),TempC1,TempC2,Pres(hPa),Lat,Lon,Sats,MOSFET,Checksum");
      logfile.flush();
    }
  }
}

void loop() {
  // Update GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Blink GPS LED: blink if no fix, solid if fix
  static unsigned long lastBlink = 0;
  if (gps.location.isValid() && gps.altitude.isValid() && gps.satellites.value() > 0) {
    digitalWrite(GPS_LED, HIGH);
  } else {
    if (millis() - lastBlink > 500) {
      digitalWrite(GPS_LED, !digitalRead(GPS_LED));
      lastBlink = millis();
    }
  }

  // Read sensors
  sensors.requestTemperatures();
  float tempC1 = sensors.getTempCByIndex(0);
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

  // Build telemetry sentence
  char sentence[180];
  snprintf(sentence, sizeof(sentence),
           "$$HAB1,%lu,%.1f,%.1f,%.1f,%.1f,%.6f,%.6f,%d,%s",
           sentenceCounter++,
           alt_m,
           tempC1,
           tempC2,
           pressure,
           gps.location.isValid() ? gps.location.lat() : 0.0,
           gps.location.isValid() ? gps.location.lng() : 0.0,
           gps.satellites.value(),
           eventStr.c_str());

  // Add checksum
  unsigned char checksum = 0;
  for (size_t i = 2; i < strlen(sentence); i++) {
    checksum ^= (unsigned char)sentence[i];
  }
  char finalSentence[200];
  snprintf(finalSentence, sizeof(finalSentence), "%s*%02X", sentence, checksum);

  // Print to Serial
  Serial.println(finalSentence);

  // Write to SD card (CSV-friendly)
  if (logfile) {
    logfile.print("$$HAB1,");
    logfile.print(sentenceCounter - 1);
    logfile.print(",");
    logfile.print(alt_m, 1);
    logfile.print(",");
    logfile.print(tempC1, 1);
    logfile.print(",");
    logfile.print(tempC2, 1);
    logfile.print(",");
    logfile.print(pressure, 1);
    logfile.print(",");
    logfile.print(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
    logfile.print(",");
    logfile.print(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
    logfile.print(",");
    logfile.print(gps.satellites.value());
    logfile.print(",");
    logfile.print(eventStr);
    logfile.print(",");//replased line
    //logfile.println(checksum, HEX);

    logfile.flush();
  }

  // Transmit RTTY
  rtty_txstring(finalSentence);
  rtty_txbyte('\n');  // clean end of transmission
  delay(100);
}

// --- RTTY TX ---
void rtty_txstring(const char *string) {
  while (*string) {
    rtty_txbyte(*string++);
  }
}

void rtty_txbyte(char c) {
  // Start bit
  rtty_txbit(0);

  // Data bits, LSB first
  for (int i = 0; i < 7; i++) {
    rtty_txbit(c & 1);
    c >>= 1;
  }

  // Stop bits (2)
  rtty_txbit(1);
  rtty_txbit(1);
}

void rtty_txbit(int bit) {
  digitalWrite(RTTY_PIN, bit);
  digitalWrite(RTTY_LED, bit ? LOW : HIGH);  // LED flashes with bit timing
  delayMicroseconds(20000);  // ~50 baud
}

// --- HIGH-ALTITUDE GPS PATCH ---
void sendGPSAirborneMode() {
  const uint8_t cfgNav5[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
    0xFF, 0xFF,
    0x06,
    0x03,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05,
    0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
    0x16, 0xDC
  };

  while (Serial1.available()) (void)Serial1.read();
  for (size_t i = 0; i < sizeof(cfgNav5); i++) {
    Serial1.write((uint8_t)cfgNav5[i]);
  }
  delay(100);
}
