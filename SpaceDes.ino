#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include "MAX471.h"

// Create instances of the sensors
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;

// Raw data for the magnetometer
int16_t mx, my, mz;

// Raw sensor data for MPU6050
int16_t ax, ay, az, gx, gy, gz;

// Max471 test parameters
#define TEST_DELAY delay(2000);
#define VT_PIN A0
#define AT_PIN A1

// Initialize MAX471 sensor (using 10-bit ADC, Vcc from battery, and pins for current and voltage measurement)
MAX471 myMax471(ADC_10_bit , VCC_BAT, AT_PIN, VT_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize magnetometer
  if (!mag.begin()) {
    while (1);
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    while (1);
  }

  // Set ranges for sensors
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Initialize MAX471 sensor
  Serial.println("== START COMBINED SENSOR TEST ==");
}

void loop() {
  // Read data from HMC5883L (magnetometer)
  readRawMagnetometer();
  
  // Read data from MPU6050 (accelerometer and gyroscope)
  readRawMPU6050();

  // Convert magnetometer and MPU6050 data to binary and hex
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

  String combinedHex = binaryToHex(combinedBinary);

  // Read data from MAX471 (current and voltage)
  float current = myMax471.CurrentRead();
  float voltage = myMax471.VoltageRead();
  
  // Convert current and voltage data to binary and hex
  String currentBinary = formatToBinary(current, "current");
  String voltageBinary = formatToBinary(voltage, "voltage");
  String max471CombinedBinary = voltageBinary + currentBinary;
  String max471CombinedHex = String(strtol(max471CombinedBinary.c_str(), NULL, 2), HEX);

  // Combine both the hex strings
  String finalCombinedHex = combinedHex + max471CombinedHex;

  // Print the final combined hex
  Serial.print("Combined Hex: ");
  Serial.println(finalCombinedHex);  // This prints the final long hex string

  TEST_DELAY;
}

// Read raw data from HMC5883L
void readRawMagnetometer() {
  sensors_event_t event;
  mag.getEvent(&event);
  mx = event.magnetic.x;
  my = event.magnetic.y;
  mz = event.magnetic.z;
}

// Read raw data from MPU6050
void readRawMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temp
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
  return raw / 131.0;
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

// Function to convert a floating point value to binary with specified bit allocation
String formatToBinary(float value, String type) {
  int wholePart;
  int decimalPart;

  // Voltage takes 4 bits for the whole number and 7 bits for the decimal part
  if (type == "voltage") {
    wholePart = (int)value;  // Get the whole number part (max 8)
    decimalPart = (int)((value - wholePart) * 100);  // Get the first 2 decimal places (scaled by 100)
  }
  // Current takes 2 bits for the whole number and 7 bits for the decimal part
  else if (type == "current") {
    wholePart = (int)value;  // Get the whole number part (max 3)
    decimalPart = (int)((value - wholePart) * 100);  // Get the first 2 decimal places (scaled by 100)
  }

  // Convert to binary
  String wholeBinary = String(wholePart, BIN);
  String decimalBinary = String(decimalPart, BIN);

  // Ensure binary formats are padded to the required bit length
  if (type == "voltage") {
    // Voltage: 4 bits for whole part, 7 bits for decimal
    wholeBinary = wholeBinary.length() > 4 ? wholeBinary.substring(wholeBinary.length() - 4) : wholeBinary;
    decimalBinary = decimalBinary.length() > 7 ? decimalBinary.substring(decimalBinary.length() - 7) : decimalBinary;
    while (wholeBinary.length() < 4) {
      wholeBinary = "0" + wholeBinary;
    }
    while (decimalBinary.length() < 7) {
      decimalBinary = "0" + decimalBinary;
    }
  }
  else if (type == "current") {
    // Current: 2 bits for whole part, 7 bits for decimal
    wholeBinary = wholeBinary.length() > 2 ? wholeBinary.substring(wholeBinary.length() - 2) : wholeBinary;
    decimalBinary = decimalBinary.length() > 7 ? decimalBinary.substring(decimalBinary.length() - 7) : decimalBinary;
    while (wholeBinary.length() < 2) {
      wholeBinary = "0" + wholeBinary;
    }
    while (decimalBinary.length() < 7) {
      decimalBinary = "0" + decimalBinary;
    }
  }

  // Combine and return the final binary string
  return wholeBinary + decimalBinary;
}
