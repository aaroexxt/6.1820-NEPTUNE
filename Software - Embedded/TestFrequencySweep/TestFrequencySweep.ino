// Import default libraries
#include <Arduino.h>
#include "pindefs.h" // Pin definitions

void setup() {
  delay(500);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HYDROPHONE_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
}

int f = 8500; // Starting freq
int d = 1;

void loop() {
  tone(HYDROPHONE_PIN, f);
  f += d;
  if ((f > 15000 && d == 1) || (f < 7500 && d == -1)) {
    d *= -1;
  }
  if (f % 3 == 0) delay(1);
}