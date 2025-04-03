#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Constants for frequency and timing
#define FFT_BIN_WIDTH 43.0664
#define MESSAGE_BIT_DELAY 250
#define MESSAGE_START_FREQ 20000
#define MESSAGE_0_FREQ 17500
#define MESSAGE_1_FREQ 15000
#define MESSAGE_LOWPASS_CUTOFF_FREQ 10000
#define FFT_BIN_CUTOFF (int)(MESSAGE_LOWPASS_CUTOFF_FREQ / FFT_BIN_WIDTH)
#define MIN_VALID_AMP 0.1
#define MAX_VALID_AMP 1.5
#define BOUNDS_FREQ 1500

// UnderwaterMessage C struct definition (we can change this)
union UnderwaterMessage {
    struct {
        uint8_t msg : 3; // 3 bits for message
        uint8_t id : 4;  // 4 bits for ID
    };
    uint8_t data; // Combined 7 bits into 1 byte

    static constexpr uint8_t size = 7;
};

// Audio Components Initialization
/*
For explanation: the SGTL5000 chip is used for generating and receiving audio (message out tones and message in tones)
See Datasheet here: https://www.nxp.com/docs/en/data-sheet/SGTL5000.pdf
essentially, it can be used in many different modes, the code below just makes a simple structure of:asm

WATER INPUT -> analog amplifier -> digital amplifier -> Fourier transform (to get frequency space spectrogram) -> User code

User code (message we want to send) -> output (simple!)
*/
AudioControlSGTL5000 audioShield;
AudioInputI2S audioInput;
AudioAmplifier inputAmp;
AudioAnalyzeFFT1024 inputFFT;
AudioConnection patchCord1(audioInput, 0, inputAmp, 0);
AudioConnection patchCord2(inputAmp, 0, inputFFT, 0);

// State Machine Definition
// these are just different states we can be in
enum RECEIVING_STATE {
    LISTENING, // waiting for the "start message" tone
    CHECK_START, // we saw "start message" for at least 1 millisecond, does it stay valid for a full bit?
    MESSAGE_ACTIVE // currently reading a message!
};

RECEIVING_STATE state = LISTENING;

// Buffers and Variables
#define NUM_SAMPLES ((int)(((MESSAGE_BIT_DELAY / 10.0) * (86.0 / 100.0)) + 1.0 + 0.9999)) // just how many samples we can take with the FFT in a single bit window
int16_t samplingBuffer[NUM_SAMPLES];
uint16_t samplingPointer = 0;
bool sampling = false;
bool bitBuffer[UnderwaterMessage::size];
int bitPointer = 0;
unsigned long lastBitChange = 0;

void setup() {
    Serial.begin(115200);

    AudioMemory(12);

    // Setup SGTL5000
    audioShield.enable();
    audioShield.inputSelect(AUDIO_INPUT_MIC);
    audioShield.micGain(60);

    inputAmp.gain(2);

    clearSampleBuffer();
    clearBitBuffer();
}

void loop() {
    if (inputFFT.available()) {
        float maxBinAmp = 0;
        int binNumber = 0;

        // Find strongest frequency above cutoff
        for (int i = FFT_BIN_CUTOFF; i < 1024; i++) {
            float amp = inputFFT.read(i);
            if (amp > maxBinAmp) {
                maxBinAmp = amp;
                binNumber = i;
            }
        }

        double detectedFreq = binNumber * FFT_BIN_WIDTH;
// State machine handling
switch (state) {

    case LISTENING:
        // Waiting for a valid message start frequency
        if (validAmplitude(maxBinAmp) && freqMatchesBounds(detectedFreq, BOUNDS_FREQ, MESSAGE_START_FREQ)) {
            // Valid amplitude and start frequency detected, begin verifying start signal
            transitionState(CHECK_START);
        }
        break;

    case CHECK_START:
        // Confirming we've consistently detected the start frequency over the sampling period
        if (millis() - lastBitChange >= MESSAGE_BIT_DELAY) {
            if (isSampleBufferValid() && freqMatchesBounds(sampleBufferMax(), BOUNDS_FREQ, MESSAGE_START_FREQ)) {
                // Confirmed valid start frequency, proceed to actively receive message bits
                transitionState(MESSAGE_ACTIVE);
            } else {
                // Start frequency was not consistent, return to LISTENING state
                transitionState(LISTENING);
            }
        }
        break;

    case MESSAGE_ACTIVE:
        // Actively receiving bits of the message at regular intervals
        if (millis() - lastBitChange >= MESSAGE_BIT_DELAY) {
            if (isSampleBufferValid()) {
                double bufferFreq = sampleBufferMax();

                // Determine if the bit received is '1' or '0' based on frequency detected
                if (freqMatchesBounds(bufferFreq, BOUNDS_FREQ, MESSAGE_1_FREQ)) {
                    bitBuffer[bitPointer] = 1;  // Bit is '1'
                } else if (freqMatchesBounds(bufferFreq, BOUNDS_FREQ, MESSAGE_0_FREQ)) {
                    bitBuffer[bitPointer] = 0;  // Bit is '0'
                }
            }

            clearSampleBuffer();
            bitPointer++;  // Advance to the next bit
            lastBitChange = millis();

            // Check if all bits of the message have been received
            if (bitPointer >= UnderwaterMessage::size) {
                UnderwaterMessage msg;
                msg.data = 0;

                // Reconstruct the message from individual bits
                for (int i = UnderwaterMessage::size - 1; i >= 0; i--) {
                    msg.data = (msg.data << 1) | bitBuffer[i];
                }

                // Output the decoded message details
                Serial.print("Received Message ID: ");
                Serial.print(msg.id);
                Serial.print(" MSG: ");
                Serial.println(msg.msg);

                // Reset state back to LISTENING for next message
                transitionState(LISTENING);
            }
        }
        break;

        // If sampling, record the bin
        if (sampling && validAmplitude(maxBinAmp)) {
            samplingBuffer[samplingPointer++] = binNumber;
            if (samplingPointer >= NUM_SAMPLES) samplingPointer = 0;
        }
    }
}

// Helper function to transition states
void transitionState(RECEIVING_STATE newState) {
    state = newState;
    sampling = (state != LISTENING);
    clearSampleBuffer();
    clearBitBuffer();
    lastBitChange = millis();
}

// Helper Functions

// Is a given amplitude valid? (simple check)
bool validAmplitude(float amp) {
    return amp >= MIN_VALID_AMP && amp <= MAX_VALID_AMP;
}

// Is a frequency within a certain range (bounds) of a target?
bool freqMatchesBounds(double freq, double bounds, double target) {
    return abs(freq - target) <= (bounds / 2.0);
}

// Do more then 1/3 of our samples have a valid frequency?
bool isSampleBufferValid() {
    int count = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        if (samplingBuffer[i] != -1) count++;
    }
    return count >= (NUM_SAMPLES / 3);
}

// Get the most commonly occurring frequency in the sample buffer (its mode)
double sampleBufferMax() {
    int hist[1024] = {0}, maxCount = 0, maxBin = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) hist[samplingBuffer[i]]++;
    for (int i = 0; i < 1024; i++) {
        if (hist[i] > maxCount) {
            maxCount = hist[i];
            maxBin = i;
        }
    }
    return maxBin * FFT_BIN_WIDTH;
}

// just reset sample buffer lol - set to 0
void clearSampleBuffer() {
    for (int i = 0; i < NUM_SAMPLES; i++) samplingBuffer[i] = -1;
    samplingPointer = 0;
}

// same thing for the bit buffer
void clearBitBuffer() {
    for (int i = 0; i < UnderwaterMessage::size; i++) bitBuffer[i] = 0;
    bitPointer = 0;
}
