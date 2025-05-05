#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3); // RX, TX

// Helper function: Convert integer to binary string with fixed length
String toBinary(unsigned long val, int bits) {
  String s = "";
  for (int i = bits - 1; i >= 0; i--) {
    s += (val >> i) & 1;
  }
  return s;
}

// ========== TIME ENCODING ==========
String encodeTime(uint8_t month, uint8_t day, uint8_t year, uint8_t hour, uint8_t minute, uint8_t second) {
  return toBinary(month, 4) +
         toBinary(day, 5) +
         toBinary(year, 6) +
         toBinary(hour, 5) +
         toBinary(minute, 6) +
         toBinary(second, 6);
}

// ========== LAT/LON ENCODING ==========
String encodeCoordinate(float coord) {
  String bin = "";

  // Sign bit
  bin += (coord < 0) ? "1" : "0";
  if (coord < 0) coord = -coord;

  // Whole part
  int whole = int(coord);
  bin += toBinary(whole, 8);

  // Decimal part (as direct integer)
  unsigned long decimal = (coord - whole) * 10000000UL;
  if (decimal > 0x7FFFFF) decimal = 0x7FFFFF;
  bin += toBinary(decimal, 23);

  return bin;
}

float decodeCoordinate(String bin) {
  int sign = (bin[0] == '1') ? -1 : 1;
  int whole = strtol(bin.substring(1, 9).c_str(), nullptr, 2);
  unsigned long decimalPart = strtoul(bin.substring(9).c_str(), nullptr, 2);
  float decimal = decimalPart / 10000000.0;
  return sign * (whole + decimal);
}

// ========== ALTITUDE ENCODING ==========
String encodeAltitude(float altitude) {
  String bin = "";

  bin += (altitude < 0) ? "1" : "0";
  if (altitude < 0) altitude = -altitude;

  unsigned int whole = (unsigned int)altitude;
  float decimal = altitude - whole;
  unsigned int decimalInt = (unsigned int)(decimal * 100); // 2 decimal digits
  if (decimalInt > 127) decimalInt = 127;

  bin += toBinary(whole, 16);
  bin += toBinary(decimalInt, 7);

  return bin;
}

float decodeAltitude(String bin) {
  int sign = (bin[0] == '1') ? -1 : 1;
  int whole = strtol(bin.substring(1, 17).c_str(), nullptr, 2);
  int decimalInt = strtol(bin.substring(17).c_str(), nullptr, 2);
  float decimal = decimalInt / 100.0;
  return sign * (whole + decimal);
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated()) {
      // Get time components
      uint8_t month = gps.date.month();
      uint8_t day = gps.date.day();
      uint8_t year = gps.date.year() - 2000;
      uint8_t hour = gps.time.hour();
      uint8_t minute = gps.time.minute();
      uint8_t second = gps.time.second();

      // Get coordinates and altitude
      float lat = gps.location.lat();
      float lon = gps.location.lng();
      float alt = gps.altitude.meters();

      // Encode all data
      String timeBin = encodeTime(month, day, year, hour, minute, second);
      String latBin = encodeCoordinate(lat);
      String lonBin = encodeCoordinate(lon);
      String altBin = encodeAltitude(alt);

      // Decode to check
      float decodedLat = decodeCoordinate(latBin);
      float decodedLon = decodeCoordinate(lonBin);
      float decodedAlt = decodeAltitude(altBin);

      // === OUTPUT ===
      Serial.println("========== PACKET ==========");

      Serial.print("Time: ");
      Serial.print(month); Serial.print("/");
      Serial.print(day); Serial.print("/");
      Serial.print(2000 + year); Serial.print(" ");
      Serial.print(hour); Serial.print(":");
      Serial.print(minute); Serial.print(":");
      Serial.println(second);

      Serial.print("Time Bin: "); Serial.println(timeBin);
      Serial.print("Time Hex: "); Serial.println(String(strtoul(timeBin.c_str(), nullptr, 2), HEX));

      Serial.println();

      Serial.print("Lat: "); Serial.println(lat, 7);
      Serial.print("Lat Bin: "); Serial.println(latBin);
      Serial.print("Lat Hex: "); Serial.println(String(strtoul(latBin.c_str(), nullptr, 2), HEX));
      Serial.print("Decoded Lat: "); Serial.println(decodedLat, 7);

      Serial.println();

      Serial.print("Lon: "); Serial.println(lon, 7);
      Serial.print("Lon Bin: "); Serial.println(lonBin);
      Serial.print("Lon Hex: "); Serial.println(String(strtoul(lonBin.c_str(), nullptr, 2), HEX));
      Serial.print("Decoded Lon: "); Serial.println(decodedLon, 7);

      Serial.println();

      Serial.print("Alt: "); Serial.print(alt); Serial.println(" m");
      Serial.print("Alt Bin: "); Serial.println(altBin);
      Serial.print("Alt Hex: "); Serial.println(String(strtoul(altBin.c_str(), nullptr, 2), HEX));
      Serial.print("Decoded Alt: "); Serial.print(decodedAlt); Serial.println(" m");

      Serial.println("============================\n");

      delay(3000);
    }
  }
}
