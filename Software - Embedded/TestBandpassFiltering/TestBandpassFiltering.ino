#include <Arduino.h>
#include <Audio.h>
#include "pindefs.h"

// === Frequencies + Sample Settings ===
#define MESSAGE_START_FREQ 20000
#define MESSAGE_0_FREQ     17500
#define MESSAGE_1_FREQ     15000
#define MESSAGE_END_FREQ   22000
#define BANDWIDTH_FREQ     500 // Hz
#define MIN_VALID_AMP      2000
#define MAX_VALID_AMP      8000

// === Audio Setup ===
const int micInput = AUDIO_INPUT_MIC;
const uint32_t sampleRate = 44100;

// === Audio Objects ===
AudioInputI2S        audioInput;
AudioAmplifier       inputAmp;
AudioControlSGTL5000 audioShield;

// === BandpassBranch Struct ===
struct BandpassBranch {
  AudioFilterBiquad* filter;
  AudioRecordQueue* queue;
  AudioConnection* inputToFilter;
  AudioConnection* filterToQueue;
};

// === Bandpass Branch Instances ===
BandpassBranch* branchF_START;
BandpassBranch* branchF_0;
BandpassBranch* branchF_1;
BandpassBranch* branchF_END;

// === Create Bandpass Branch ===
BandpassBranch* createBandpassBranch(AudioStream& input,
                                     float sampleRate,
                                     float centerFreq,
                                     float bandwidth,
                                     int stages = 4) {
  BandpassBranch* branch = new BandpassBranch;
  branch->filter = new AudioFilterBiquad();
  branch->queue = new AudioRecordQueue();

  float Q_total = centerFreq / bandwidth;
  float Q_stage = Q_total / sqrt((float)stages);

  for (int i = 0; i < stages; i++) {
    branch->filter->setBandpass(i, centerFreq, Q_stage);
  }

  branch->inputToFilter = new AudioConnection(input, 0, *branch->filter, 0);
  branch->filterToQueue = new AudioConnection(*branch->filter, 0, *branch->queue, 0);

  // Start recording!
  *branch->queue->begin();

  return branch;
}

// === Sample Reading ===
double getQueueAverageAmplitude(AudioRecordQueue* queue) {
    float averageAmplitude = 0;
    int samples = 0;
    while (queue->available() > 0) {
    int16_t* data = queue->readBuffer();
    if (!data) continue;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        averageAmplitude += (double)abs(data[i]); // Normalize
        samples++;
    }
    queue->freeBuffer();
  }
  return averageAmplitude/(double)samples;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  AudioMemory(60);
  inputAmp.gain(2);

  // Initialize SGTL5000
  audioShield.enable();
  audioShield.inputSelect(micInput);
  audioShield.micGain(90);
  audioShield.volume(1);
  
  // Create branches for each frequency (autostarts queue)
  branchF_START = createBandpassBranch(inputAmp, sampleRate, MESSAGE_START_FREQ, BANDWIDTH_FREQ);
  branchF_0 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_0_FREQ, BANDWIDTH_FREQ);
  branchF_1 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_1_FREQ, BANDWIDTH_FREQ);
  branchF_END = createBandpassBranch(inputAmp, sampleRate, MESSAGE_END_FREQ, BANDWIDTH_FREQ);

  // Label output columns for Serial Plotter
  Serial.println("StartFreq\tFreq0\tFreq1\tEndFreq");
}

void loop() {
  // Read average sample amplitudes in each filter
  double startAmp = getQueueAverageAmplitude(branchF_START->queue);
  double f0Amp    = getQueueAverageAmplitude(branchF_0->queue);
  double f1Amp    = getQueueAverageAmplitude(branchF_1->queue);
  double endAmp   = getQueueAverageAmplitude(branchF_END->queue);

  // Plot results
  Serial.print(startAmp);
  Serial.print("\t");
  Serial.print(f0Amp);
  Serial.print("\t");
  Serial.print(f1Amp);
  Serial.print("\t");
  Serial.println(endAmp);

  delay(50); // Smooth plot
}
