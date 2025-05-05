/*
* Project Name: MAX471 with Binary, Hexadecimal, and Actual Value
* File: MAX471_BinaryFormat.ino
* Description: Library for MAX471 with binary, hexadecimal, and actual value formatting example file
* Tested: on UNO / NANO ATmega328
* URL: https://github.com/gavinlyonsrepo/MAX471
*/


//******************** LIBRARIES ******************
#include "MAX471.h"

//******************** GLOBALS + DEFINES **********
// Max471 test parameters
#define TEST_DELAY delay(2000);
#define VT_PIN A0
#define AT_PIN A1

// Initialize MAX471 sensor (using 10-bit ADC, Vcc from battery, and pins for current and voltage measurement)
MAX471 myMax471(ADC_10_bit , VCC_BAT, AT_PIN, VT_PIN);

//******************** SETUP *********************
void setup() {
  Serial.begin(38400);
  Serial.println("== START LIBRARY TEST WITH BINARY, HEX, AND ACTUAL VALUE ==");
}

//******************* MAIN LOOP ******************
void loop() {

  // Test 1: Read Amps and Volts
  float current = myMax471.CurrentRead();  // Read current in Amps
  float voltage = myMax471.VoltageRead();  // Read voltage in Volts
  
  // Convert the readings to binary, hexadecimal, and actual values
  String currentBinary = formatToBinary(current, "current");
  String voltageBinary = formatToBinary(voltage, "voltage");
  
  // Combine the binary values for current and voltage
  String combinedBinary = voltageBinary + currentBinary;
  
  // Convert the combined binary value to hexadecimal
  String combinedHex = String(strtol(combinedBinary.c_str(), NULL, 2), HEX);

  // Print the values in binary, hexadecimal, and actual form
  Serial.print("Current (Binary): ");
  Serial.println(currentBinary);
  Serial.print("Voltage (Binary): ");
  Serial.println(voltageBinary);
  
  Serial.print("Combined Binary: ");
  Serial.println(combinedBinary);
  Serial.print("Combined Hex: ");
  Serial.println(combinedHex);
  
  Serial.print("Current (Actual): ");
  Serial.println(current, 3);  // Print actual value with 3 decimal places
  Serial.print("Voltage (Actual): ");
  Serial.println(voltage, 3);  // Print actual value with 3 decimal places
  
  Serial.println();
  TEST_DELAY

} // End of main loop

//******************** HELPER FUNCTIONS ********************

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

//******************* EOF *****************
