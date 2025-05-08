#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include "MAX471.h"

// Create sensor instances
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3); // RX, TX

// Raw data containers
int16_t mx, my, mz;
int16_t ax, ay, az, gx, gy, gz;

#define VT_PIN A0
#define AT_PIN A1
MAX471 myMax471(ADC_10_bit , VCC_BAT, AT_PIN, VT_PIN);

// Helper functions
String toBinary(unsigned long val, int bits) {
  String s = "";
  for (int i = bits - 1; i >= 0; i--) s += (val >> i) & 1;
  return s;
}

String padHex(const String& hexStr, int width) {
  String s = hexStr;
  while (s.length() < width) s = "0" + s;
  return s;
}

String encodeTime(uint8_t mo, uint8_t d, uint8_t y, uint8_t h, uint8_t m, uint8_t s) {
  return toBinary(mo,4) + toBinary(d,5) + toBinary(y,6)
       + toBinary(h,5) + toBinary(m,6) + toBinary(s,6);
}

String encodeCoordinate(float coord) {
  String b = "";
  b += (coord < 0) ? '1' : '0';
  if (coord < 0) coord = -coord;
  int whole = int(coord);
  b += toBinary(whole,8);
  unsigned long frac = (coord - whole) * 10000000UL;
  if (frac > 0x7FFFFF) frac = 0x7FFFFF;
  b += toBinary(frac,23);
  return b;
}

String encodeAltitude(float alt) {
  String b = "";
  b += (alt < 0) ? '1' : '0';
  if (alt < 0) alt = -alt;
  unsigned int w = (unsigned int)alt;
  unsigned int f = (unsigned int)((alt - w) * 100);
  if (f > 127) f = 127;
  b += toBinary(w,16);
  b += toBinary(f,7);
  return b;
}

void readRawMagnetometer() {
  sensors_event_t event;
  mag.getEvent(&event);
  mx = event.magnetic.x;
  my = event.magnetic.y;
  mz = event.magnetic.z;
}

void readRawMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

float convertToFloatMagnetometer(int16_t raw) {
  return raw * 0.92;
}

float convertToFloatAccel(int16_t raw) {
  return (raw / 16384.0) * 9.81;
}

float convertToFloatGyro(int16_t raw) {
  float degPerSec = raw / 131.0;
  return degPerSec * (PI / 180.0);
}

byte convertToSignedScaledByte(float value, float scale = 3.0) {
  bool isNegative = value < 0;
  float absVal = abs(value);
  int scaled = round(absVal * scale);
  scaled = constrain(scaled, 0, 127);
  return (isNegative << 7) | (scaled & 0x7F);
}

String collectBinary(int16_t rawData, float convertedData) {
  byte binVal = convertToSignedScaledByte(convertedData);
  String binaryStr = "";
  for (int i = 7; i >= 0; i--) {
    binaryStr += String(bitRead(binVal, i));
  }
  return binaryStr;
}

String binaryToHex(String binary) {
  String hex = "";
  int length = binary.length();
  for (int i = 0; i < length; i += 4) {
    String chunk = binary.substring(i, i + 4);
    hex += String(strtol(chunk.c_str(), NULL, 2), HEX);
  }
  return hex;
}

String formatToBinary(float value, String type) {
  int wholePart = (int)value;
  int decimalPart = (int)((value - wholePart) * 100);
  String wholeBinary = String(wholePart, BIN);
  String decimalBinary = String(decimalPart, BIN);

  if (type == "voltage") {
    while (wholeBinary.length() < 4) wholeBinary = "0" + wholeBinary;
    while (decimalBinary.length() < 7) decimalBinary = "0" + decimalBinary;
    return wholeBinary + decimalBinary;
  } else {
    while (wholeBinary.length() < 2) wholeBinary = "0" + wholeBinary;
    while (decimalBinary.length() < 7) decimalBinary = "0" + decimalBinary;
    return wholeBinary + decimalBinary;
  }
}

String getCombinedHex() {
  readRawMagnetometer();
  readRawMPU6050();

  String combinedBinary = "";
  combinedBinary += collectBinary(mx, convertToFloatMagnetometer(mx));
  combinedBinary += collectBinary(my, convertToFloatMagnetometer(my));
  combinedBinary += collectBinary(mz, convertToFloatMagnetometer(mz));
  combinedBinary += collectBinary(ax, convertToFloatAccel(ax));
  combinedBinary += collectBinary(ay, convertToFloatAccel(ay));
  combinedBinary += collectBinary(az, convertToFloatAccel(az));
  combinedBinary += collectBinary(gx, convertToFloatGyro(gx));
  combinedBinary += collectBinary(gy, convertToFloatGyro(gy));
  combinedBinary += collectBinary(gz, convertToFloatGyro(gz));

  float current = myMax471.CurrentRead();
  float voltage = myMax471.VoltageRead();
  String currentBinary = formatToBinary(current, "current");
  String voltageBinary = formatToBinary(voltage, "voltage");
  combinedBinary += voltageBinary + currentBinary;

  if (gps.location.isUpdated() && gps.altitude.isUpdated()
      && gps.date.isUpdated() && gps.time.isUpdated()) {
    uint8_t mo = gps.date.month();
    uint8_t d  = gps.date.day();
    uint8_t y  = gps.date.year() - 2000;
    uint8_t h  = gps.time.hour();
    uint8_t mi = gps.time.minute();
    uint8_t se = gps.time.second();
    float lat = gps.location.lat();
    float lon = gps.location.lng();
    float alt = gps.altitude.meters();

    String tBin = encodeTime(mo,d,y,h,mi,se);
    String latBin = encodeCoordinate(lat);
    String lonBin = encodeCoordinate(lon);
    String altBin = encodeAltitude(alt);

    combinedBinary += tBin + latBin + lonBin + altBin;
  }

  return binaryToHex(combinedBinary);
}

unsigned long lastPrint = 0;
const unsigned long interval = 3000;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();

  if (!mag.begin()) while (1);
  if (!mpu.begin()) while (1);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("== START SENSOR + GPS COMBINED ==");
}

void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (millis() - lastPrint >= interval) {
    lastPrint = millis();
    String combinedHex = getCombinedHex();
    Serial.print("Combined Hex: "); Serial.println(combinedHex);
    Serial.println("---");
  }
}
