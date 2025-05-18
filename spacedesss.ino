#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include "MAX471.h"

SdFat SD;
const uint8_t CS_PIN = 10;
SoftwareSerial gpsSerial(2, 3); // RX, TX for GPS
TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag(12345);
Adafruit_MPU6050 mpu;
MAX471 myMax(ADC_10_bit, VCC_BAT, A1, A0);

uint8_t pkt[27];
char hexOut[55];
unsigned long lastT;

void pack(uint32_t v, int b, int &p) {
  while (b--) {
    int i = p >> 3, bit = 7 - (p & 7);
    pkt[i] |= ((v >> b) & 1) << bit;
    p++;
  }
}

void build() {
  memset(pkt, 0, sizeof(pkt));
  int p = 0;
  sensors_event_t e;
  mag.getEvent(&e);
  float ms[3] = {e.magnetic.x * 0.92, e.magnetic.y * 0.92, e.magnetic.z * 0.92};
  for (int i = 0; i < 3; i++) {
    pack(ms[i] < 0, 1, p);
    pack(min(127, int(abs(ms[i]) * 3)), 7, p);
  }

  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  int16_t d[6];
  for (int i = 0; i < 3; i++) {
    d[i] = (Wire.read() << 8) | Wire.read(); Wire.read(); Wire.read();
  }
  for (int i = 3; i < 6; i++) d[i] = (Wire.read() << 8) | Wire.read();
  float ss[6] = {9.81 / 16384.0, 9.81 / 16384.0, 9.81 / 16384.0, PI / 180.0 / 131.0, PI / 180.0 / 131.0, PI / 180.0 / 131.0};
  for (int i = 0; i < 6; i++) {
    float v = d[i] * ss[i];
    pack(v < 0, 1, p);
    pack(min(127, int(abs(v) * 3)), 7, p);
  }

  float c = myMax.CurrentRead(), v = myMax.VoltageRead();
  pack(uint32_t(v), 4, p); pack(uint32_t((v - int(v)) * 100), 7, p);
  pack(uint32_t(c), 2, p); pack(uint32_t((c - int(c)) * 100), 7, p);

  if (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isUpdated()) {
    pack(gps.date.month(), 4, p); pack(gps.date.day(), 5, p); pack(gps.date.year() - 2000, 6, p);
    pack(gps.time.hour(), 5, p); pack(gps.time.minute(), 6, p); pack(gps.time.second(), 6, p);
    auto ec = [&](double x, int w, int f) {
      pack(x < 0, 1, p); if (x < 0) x = -x;
      pack(uint32_t(x), w, p); pack(uint32_t((x - int(x)) * (1UL << f)), f, p);
    };
    ec(gps.location.lat(), 8, 23); ec(gps.location.lng(), 8, 23); ec(gps.altitude.meters(), 16, 7);
  }
}

void toHex() {
  for (int i = 0; i < 27; i++) {
    uint8_t b = pkt[i];
    hexOut[2 * i] = (b >> 4) < 10 ? ('0' + (b >> 4)) : ('A' + (b >> 4) - 10);
    hexOut[2 * i + 1] = (b & 0x0F) < 10 ? ('0' + (b & 0x0F)) : ('A' + (b & 0x0F) - 10);
  }
  hexOut[54] = 0;
}

// Servo control variables
const int servo1Pin = 4;
const int servo2Pin = 5;
String input = "";
unsigned long actionTime = 0;
byte state = 0; // 0: idle, 1: unlocking, 2: deploying

void moveServo(int pin, int angle) {
  int pw = map(angle, 0, 180, 544, 2400);
  for (int i = 0; i < 50; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(pw);
    digitalWrite(pin, LOW);
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin(); SPI.begin();
  mag.begin(); mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  SD.begin(CS_PIN);

  pinMode(servo1Pin, OUTPUT);
  pinMode(servo2Pin, OUTPUT);
  moveServo(servo1Pin, 0);
  moveServo(servo2Pin, 90);
}

void loop() {
  if (millis() - lastT > 3000) {
    lastT = millis();
    build();
    toHex();
    Serial.println(hexOut);

    SdFile f;
    if (f.open("datalog.txt", O_CREAT | O_WRITE | O_APPEND)) {
      f.println(hexOut);
      f.sync();
      f.close();
    }

    char poopyOut[11];
    strcpy(poopyOut, "POOPY");
    strncpy(poopyOut + 5, hexOut + 18, 5);
    poopyOut[10] = '\0';
    Serial.println(poopyOut);
  }

  // --- Servo command handling (minimal changes) ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input == "unlock") {
        moveServo(servo1Pin, 180);
        actionTime = millis();
        state = 1;
      } else if (input == "deploy") {
        moveServo(servo2Pin, 0);
        actionTime = millis();
        state = 2;
      }
      input = "";
    } else {
      input += c;
    }
  }

  if (state == 1 && millis() - actionTime >= 1000) state = 0;
  if (state == 2 && millis() - actionTime >= 1100) {
    moveServo(servo2Pin, 90);
    state = 0;
  }
}
