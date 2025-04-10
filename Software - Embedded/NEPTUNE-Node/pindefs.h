#ifndef PINDEFS_H
#define PINDEFS_H

    // Pin connected to neopixels strip
    // THESE ARE PINS ON THE TEENSY
    #define LED_PIN  14
    #define LED_COUNT 12

    // Hydrophone output pin
    #define HYDROPHONE_PIN 3 // Which pin is the output?
    
    // Relay pin
    #define RELAY_PIN 4

    // Current ID (to be transmitted)
    uint8_t user_ID = 1;
#endif