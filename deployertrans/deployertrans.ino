#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);  // CE, CSN
const byte address[6] = "00016";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);  // Match the receiver's power level
  radio.stopListening();          // Set as transmitter
  Serial.println("Transmitter ready.");
}

void loop() {
  const char text[] = "Hello, world!";
  bool success = radio.write(&text, sizeof(text));

  if (success) {
    Serial.println("Sent: Hello, world!");
  } else {
    Serial.println("Send failed.");
  }

  delay(1000);  // Send once per second
}
