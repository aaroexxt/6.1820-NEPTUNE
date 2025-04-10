// Import default libraries
#include <Arduino.h>
#include "pindefs.h" // Pin definitions

// Import audio samples
// TODO: NEED TO COMPRESS THESE FILES
// #include "AudioAir.h"
// #include "AudioCheck_in.h"
// #include "AudioFish.h"
// #include "AudioLook.h"
//#include "AudioAscend.h"
// #include "AudioSos.h"

// audio shield SD card

// AudioPlaySdWav playSdWav;
// AudioOutputI2S audioOutput;
// AudioConnection patchCord1(playSdWav, 0, audioOutput, 0);
// AudioConnection patchCord2(playSdWav, 1, audioOutput, 1);
// AudioControlSGTL5000 audioShield;

void setup() {
  delay(500);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HYDROPHONE_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
}

int f = 7500;
int d = 1;

void loop() {
  tone(HYDROPHONE_PIN, f);
  f += d;
  if ((f > 20000 && d == 1) || (f < 7500 && d == -1)) {
    d *= -1;
  }
  if (f % 3 == 0) delay(1);
}