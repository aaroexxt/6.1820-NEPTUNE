// Protocol.h
// Author: Aaron Becker 4/13/25

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

/************* PROTOCOL DESCRIPTION */
// Defines the key characteristics of the Neptune communication protocol

// SELECT SAMPLE RATE
const uint32_t sampleRate = 44100;
//const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

#define MESSAGE_BIT_DELAY 200 // ms between bits

// in theory, we are getting 44100 samples out of bandpass per sec
#define NUM_SAMPLES (int)((MESSAGE_BIT_DELAY / 1000.0) * sampleRate)

// Center frequency is the primary communication frequency
// #define CENTER_FREQ 8000
// #define MESSAGE_START_FREQ CENTER_FREQ // Hz (1/sec)
// #define MESSAGE_0_FREQ CENTER_FREQ-500 // Hz
// #define MESSAGE_1_FREQ CENTER_FREQ+500 // Hz
// #define MESSAGE_END_FREQ CENTER_FREQ+1000 // Hz (1/sec)
//#define CENTER_FREQ 10000
#define MESSAGE_START_FREQ 7800 // Hz (1/sec)
#define MESSAGE_0_FREQ 8975 // Hz
#define MESSAGE_1_FREQ 10100 // Hz
#define MESSAGE_END_FREQ 11325 // Hz (1/sec)

// Thresholds in sample buffer length
#define THRESHOLD_SAMPLES_MIN_DETECT (int)(NUM_SAMPLES/6)
#define THRESHOLD_SAMPLES_VALID (int)(NUM_SAMPLES/4)

#endif // PROTOCOL_H