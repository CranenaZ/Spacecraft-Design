#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create instances of the sensors
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;

// Raw data for the magnetometer
int16_t mx, my, mz;

// Raw sensor data for MPU6050
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize the HMC5883L magnetometer
  if (!mag.begin()) {
    Serial.println("Couldn't find the HMC5883L sensor.");
    while (1);
  }

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  // Set the accelerometer and gyroscope ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  Serial.println("Sensors Initialized:");
  Serial.println("MPU6050 (Accelerometer: ±2g, Gyroscope: ±250°/s)");
  Serial.println("HMC5883L Magnetometer Initialized");
}

void loop() {
  // Read raw data from both the HMC5883L and MPU6050
  readRawMagnetometer();
  readRawMPU6050();

  // Combine and print Magnetometer data
  Serial.println("Magnetometer:");
  String combinedBinary = "";
  String combinedHex = "";
  
  combinedBinary += printAll("X", mx, convertToFloatMagnetometer(mx));
  combinedBinary += printAll("Y", my, convertToFloatMagnetometer(my));
  combinedBinary += printAll("Z", mz, convertToFloatMagnetometer(mz));

  // Print the direction based on magnetometer data
  String direction = decodeDirection(mx, my);
  Serial.print("Direction: ");
  Serial.println(direction);

  // Combine and print Accelerometer data
  Serial.println("Accelerometer:");
  combinedBinary += printAll("X", ax, convertToFloatAccel(ax));
  combinedBinary += printAll("Y", ay, convertToFloatAccel(ay));
  combinedBinary += printAll("Z", az, convertToFloatAccel(az));

  // Combine and print Gyroscope data
  Serial.println("Gyroscope:");
  combinedBinary += printAll("X", gx, convertToFloatGyro(gx));
  combinedBinary += printAll("Y", gy, convertToFloatGyro(gy));
  combinedBinary += printAll("Z", gz, convertToFloatGyro(gz));

  // Print combined binary
  Serial.print("Combined Binary: ");
  Serial.println(combinedBinary);

  // Convert combined binary to hexadecimal
  combinedHex = binaryToHex(combinedBinary);
  Serial.print("Combined Hex: 0x");
  Serial.println(combinedHex);
  
  Serial.println("-----------------------------");
  delay(5000);
}

// 1. Read raw data from HMC5883L magnetometer
void readRawMagnetometer() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Store the raw data
  mx = event.magnetic.x;
  my = event.magnetic.y;
  mz = event.magnetic.z;
}

// 2. Convert raw magnetometer data to microtesla (µT)
float convertToFloatMagnetometer(int16_t raw) {
  return raw * 0.92; // 0.92 µT per LSB for ±1.3G range (adjust for your specific scale factor)
}

// 3. Convert raw accelerometer data to m/s²
float convertToFloatAccel(int16_t raw) {
  return (raw / 16384.0) * 9.81; // 16384 LSB/g for ±2g
}

// 4. Convert raw gyroscope data to degrees per second (°/s)
float convertToFloatGyro(int16_t raw) {
  return raw / 131.0; // 131 LSB/°/s for ±250°/s
}

// 5. Convert signed float to 1+7 bit format (sign bit first, then value)
byte convertToSignedScaledByte(float value, float scale = 3.0) {
  bool isNegative = value < 0;
  float absVal = abs(value);

  int scaled = round(absVal * scale);
  scaled = constrain(scaled, 0, 127);  // 7-bit max

  return (isNegative << 7) | (scaled & 0x7F);
}

// 6. Convert signed byte to hex string
String convertToHex(byte val) {
  char hexStr[3];
  sprintf(hexStr, "%02X", val);
  return String(hexStr);
}

// 7. Convert signed byte to 8-bit binary string
String convertToBinary(int16_t val) {
  // Only keep the least significant byte (8 bits)
  byte lowByte = val & 0xFF;  // Get the lower byte (8 bits)

  String binaryStr = String(lowByte, BIN);
  while (binaryStr.length() < 8) {
    binaryStr = "0" + binaryStr; // Pad with leading zeros to ensure it's 8 bits
  }
  return binaryStr;
}

// 8. Decode magnetometer data into cardinal direction (North, South, East, West)
String decodeDirection(int16_t mx, int16_t my) {
  String direction = "Unknown";

  // Simple mapping based on X and Y axis
  if (mx > 0 && abs(mx) > abs(my)) {
    direction = "East";  // Positive X
  } else if (mx < 0 && abs(mx) > abs(my)) {
    direction = "West";  // Negative X
  } else if (my > 0 && abs(my) > abs(mx)) {
    direction = "North"; // Positive Y
  } else if (my < 0 && abs(my) > abs(mx)) {
    direction = "South"; // Negative Y
  }

  return direction;
}

// 9. Read raw data from MPU6050 registers
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

// 10. Decode signed scaled byte back to float
float decodeSignedScaledByte(byte data, float scale = 3.0) {
  bool isNegative = data & 0x80;
  int raw = data & 0x7F;
  float value = raw / scale;
  return isNegative ? -value : value;
}

// 11. Print data in different formats and return the binary string
String printAll(String axis, int16_t rawData, float convertedData) {
  byte binVal = convertToSignedScaledByte(convertedData);
  String hexVal = convertToHex(binVal);
  float decoded = decodeSignedScaledByte(binVal);

  String binaryStr = "";
  for (int i = 7; i >= 0; i--) {
    binaryStr += String(bitRead(binVal, i));
  }

  // Print individual data
  Serial.print(axis);
  Serial.print(": RAW=");
  Serial.print(rawData);
  Serial.print(", FLOAT=");
  Serial.print(convertedData, 2);
  Serial.print(", BIN=");
  Serial.print(binaryStr);

  Serial.print(", HEX=0x");
  Serial.print(hexVal);
  Serial.print(", DECODED=");
  Serial.println(decoded, 2);

  // Return the binary string
  return binaryStr;
}

// 12. Convert combined binary string to hexadecimal string
String binaryToHex(String binary) {
  String hex = "";
  int length = binary.length();
  for (int i = 0; i < length; i += 4) {
    String chunk = binary.substring(i, i + 4);
    hex += String(strtol(chunk.c_str(), NULL, 2), HEX);
  }
  return hex;
}
