#ifndef PINDEFS_H
#define PINDEFS_H

    // Pin connected to neopixels strip
    // THESE ARE PINS ON THE TEENSY
    #define LED_PIN  14
    #define LED_COUNT 12

    // IO expander, buttons, LEDs set up
    // THESE ARE PINS ON THE I/O EXPANDER
    #define BUTTON_PIN1 0
    #define BUTTON_PIN2 1
    #define BUTTON_PIN3 2
    #define BUTTON_PIN4 3
    #define BUTTON_PIN5 4
    #define BUTTON_PIN6 5

    // Hydrophone output pin
    #define HYDROPHONE_PIN 22 // Which pin is the output?
    
    // Relay pin
    #define RELAY_PIN 17

    // Current ID (to be transmitted)
    uint8_t user_ID = 1;
#endif