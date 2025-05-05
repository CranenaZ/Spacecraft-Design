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

  // Print Magnetometer data
  Serial.println("Magnetometer:");
  printAll("X", mx, convertToFloatMagnetometer(mx));
  printAll("Y", my, convertToFloatMagnetometer(my));
  printAll("Z", mz, convertToFloatMagnetometer(mz));

  // Print the direction based on magnetometer data
  String direction = decodeDirection(mx, my);
  Serial.print("Direction: ");
  Serial.println(direction);

  // Print Accelerometer data
  Serial.println("Accelerometer:");
  printAll("X", ax, convertToFloatAccel(ax));
  printAll("Y", ay, convertToFloatAccel(ay));
  printAll("Z", az, convertToFloatAccel(az));

  // Print Gyroscope data
  Serial.println("Gyroscope:");
  printAll("X", gx, convertToFloatGyro(gx));
  printAll("Y", gy, convertToFloatGyro(gy));
  printAll("Z", gz, convertToFloatGyro(gz));

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

// 11. Print data in different formats
void printAll(String axis, int16_t rawData, float convertedData) {
  byte binVal = convertToSignedScaledByte(convertedData);
  String hexVal = convertToHex(binVal);
  float decoded = decodeSignedScaledByte(binVal);

  Serial.print(axis);
  Serial.print(": RAW=");
  Serial.print(rawData);
  Serial.print(", FLOAT=");
  Serial.print(convertedData, 2);
  Serial.print(", BIN=");

  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(binVal, i));
  }

  Serial.print(", HEX=0x");
  Serial.print(hexVal);
  Serial.print(", DECODED=");
  Serial.println(decoded, 2);
}
