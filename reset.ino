#define RESET_PIN 3

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH); // MOSFET ON initially
}

void loop() {
  delay(300000); // 30 seconds
  digitalWrite(RESET_PIN, LOW);  // MOSFET OFF (disconnect GND)
  delay(1000);                   // 1 sec OFF
  digitalWrite(RESET_PIN, HIGH); // MOSFET ON (reconnect GND)
}