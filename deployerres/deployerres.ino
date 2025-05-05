#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00016";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, address); // Set pipe for receiving
  radio.setPALevel(RF24_PA_MIN); // Match the transmitter's power level
  radio.startListening(); // Start listening for messages
  Serial.println("Receiver ready, listening...");
}

void loop() {
  if (radio.available()) {
    char receivedMessage[32] = "";  // Create a buffer to hold the received message
    radio.read(&receivedMessage, sizeof(receivedMessage));

    // Print the received message to the Serial Monitor
    Serial.print("Received: ");
    Serial.println(receivedMessage);
  }
}
