// BandpassBranch.h
// Author: Aaron Becker 4/13/25

#ifndef BANDPASS_BRANCH_H
#define BANDPASS_BRANCH_H

#include <Arduino.h>
#include <Audio.h>

/********************* BANDPASS FILTERS */
struct BandpassBranch {
  // Keep track of the biquad filter (multi-stage used), the audio queue and audio connection objects
  AudioFilterBiquad* filter;
  AudioRecordQueue* queue;
  AudioConnection* inputToFilter;
  AudioConnection* filterToQueue;
  float centerFrequency;
  float bandwidth;
};

BandpassBranch* createBandpassBranch(AudioStream& input, float sampleRate, float centerFreq, float bandwidth, int stages = 4) {
  // Allocate a container for all parts of the branch
  BandpassBranch* branch = new BandpassBranch;
  // Assign the cFreq and bandwidth
  branch->centerFrequency = centerFreq;
  branch->bandwidth = bandwidth;

  // Allocate audio components
  branch->filter = new AudioFilterBiquad();
  branch->queue = new AudioRecordQueue();

  // Compute quality factor for each stage
  float Q_total = centerFreq / bandwidth;
  float Q_stage = Q_total / sqrt((float)stages);

  for (int i = 0; i < stages; i++) {
    branch->filter->setBandpass(i, centerFreq, Q_stage);
  }

  // Hook up audio connections
  branch->inputToFilter = new AudioConnection(input, 0, *branch->filter, 0);
  branch->filterToQueue = new AudioConnection(*branch->filter, 0, *branch->queue, 0);

  // Start recording!
  branch->queue->begin();

  return branch;
}

// Counts valid tones (ones with specific amplitude) in a given BandpassBranch buffer
int countTonePresentSamples(BandpassBranch* branch) {
    int matches = 0;
    
    // Extract fields from branch struct
    AudioRecordQueue* queue = branch->queue;
    float centerFrequency = branch->centerFrequency;
  
    while (queue->available() > 0) {
      int16_t* data = queue->readBuffer();
  
      if (!data) continue;
  
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        int16_t sample = abs(data[i]);
        if (sample >= getAmplitudeThreshold(centerFrequency)) matches++;
      }
  
      queue->freeBuffer(); // Free the buffer
    }
  
    return matches;
}

  // helper fn to get average amplitude of queue for each biquad filter queue
unsigned long sumFilterAmplitudes(BandpassBranch* branch, unsigned long* sampleCounter) {
    unsigned long sumAmplitude = 0;
    int samples = 0;
  
    // extract from struct
    AudioRecordQueue* queue = branch->queue;
  
    while (queue->available() > 0) {
        int16_t* data = queue->readBuffer();
        if (!data) continue;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            sumAmplitude += abs(data[i]);
            samples++;
        }
        queue->freeBuffer();
    }
  
    // Increment the corresponding counter (if provided)
    if (sampleCounter != nullptr) {
        *sampleCounter += samples;
    }
  
    return sumAmplitude;
}

#endif // BANDPASS_BRANCH_H