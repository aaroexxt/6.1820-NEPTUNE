/*******
 * TestBandpassFilter
 * Test of bandpass "branch" structure. Type 's' into console to enable output.
 * Author: Aaron Becker
 * 11 April 2025
 * For 6.1820
 */

#include <Arduino.h>
#include <Audio.h>
#include "pindefs.h"

//Forward decs
struct BandpassBranch; // Forward declaration (optional but clean)
unsigned long sumFilterAmplitudes(BandpassBranch* branch, unsigned long* sampleCounter);
double getQueueAverageAmplitude(AudioRecordQueue* queue, unsigned long* sampleCounter = nullptr);


/************ Frequencies + Sample Settings */
#define CENTER_FREQ 10000
#define MESSAGE_START_FREQ CENTER_FREQ // Hz (1/sec)
#define MESSAGE_0_FREQ 9000 // Hz
#define MESSAGE_1_FREQ 11000 // Hz
#define MESSAGE_END_FREQ 12000 // Hz (1/sec)
#define BANDWIDTH_FREQ     150 // Hz
#define BIT_DURATION 130

#define MAX_THRESHOLDS 10

struct FreqThreshold {
    float freq;
    float amp;
};

FreqThreshold ampThresholds[MAX_THRESHOLDS] = {
    {9000, 941.89},
    {10000, 889.67},
    {11000, 732.31},
    {12000, 747.95}
};
int numThresholds = 4;


// Returns amplitude threshold for a given frequency (Hz)
float getAmplitudeThreshold(float freqHz) {
  if (numThresholds == 0) return 1000;

  // Clamp to bounds
  if (freqHz <= ampThresholds[0].freq) return ampThresholds[0].amp;
  if (freqHz >= ampThresholds[numThresholds - 1].freq) return ampThresholds[numThresholds - 1].amp;

  // Find interval
  for (int i = 0; i < numThresholds - 1; i++) {
      float f1 = ampThresholds[i].freq;
      float f2 = ampThresholds[i + 1].freq;
      if (freqHz >= f1 && freqHz <= f2) {
          float a1 = ampThresholds[i].amp;
          float a2 = ampThresholds[i + 1].amp;
          float t = (freqHz - f1) / (f2 - f1);
          return a1 * (1 - t) + a2 * t;
      }
  }

  return 1000;
}


/************ Audio Setup */
const int micInput = AUDIO_INPUT_MIC;
const uint32_t sampleRate = 44100;

/************ Audio Objects */
AudioInputI2S        audioInput;
AudioAmplifier       inputAmp;
AudioControlSGTL5000 audioShield;
AudioConnection patchCord3(audioInput, 0, inputAmp, 0);

/************ BANDPASS FILTERS */
struct BandpassBranch {
  AudioFilterBiquad* filter;
  AudioRecordQueue* queue;
  AudioConnection* inputToFilter;
  AudioConnection* filterToQueue;
};

/************ BANDPASS BRANCHES */
BandpassBranch* branchF_START;
BandpassBranch* branchF_0;
BandpassBranch* branchF_1;
BandpassBranch* branchF_END;

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
  branch->queue->begin();

  return branch;
}

// helper fn to sum amplitude of queue for each biquad filter queue
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

// helper fn to get average amplitude of queue for each biquad filter queue
double getQueueAverageAmplitude(AudioRecordQueue* queue, unsigned long* sampleCounter) {
  double averageAmplitude = 0;
  int samples = 0;
  while (queue->available() > 0) {
      int16_t* data = queue->readBuffer();
      if (!data) continue;
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
          averageAmplitude += (double)abs(data[i]);
          samples++;
      }
      queue->freeBuffer();
  }

  // Increment the corresponding counter (if provided)
  if (sampleCounter != nullptr) {
      *sampleCounter += samples;
  }

  return (samples > 0) ? averageAmplitude / (double)samples : 0.0;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  AudioMemory(500);

  // Initialize SGTL5000
  audioShield.enable();
  audioShield.inputSelect(micInput);
  audioShield.micGain(90); // max mic (analog) gain
  audioShield.volume(1);
  inputAmp.gain(3); // amplify mic to useful range
  
  // Create branches for each frequency (autostarts queue)
  branchF_START = createBandpassBranch(inputAmp, sampleRate, MESSAGE_START_FREQ, BANDWIDTH_FREQ);
  branchF_0 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_0_FREQ, BANDWIDTH_FREQ);
  branchF_1 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_1_FREQ, BANDWIDTH_FREQ);
  branchF_END = createBandpassBranch(inputAmp, sampleRate, MESSAGE_END_FREQ, BANDWIDTH_FREQ);

  // Label output columns for Serial Plotter
  Serial.println("StartFreq\tFreq0\tFreq1\tEndFreq");
}

bool outputThresholds = false;

bool zoomMode = false;
unsigned long lastReportTime = 0;

unsigned long counter_START = 0;
unsigned long counter_0 = 0;
unsigned long counter_1 = 0;
unsigned long counter_END = 0;

unsigned long counter_THRESH_START = 0;
unsigned long counter_THRESH_0 = 0;
unsigned long counter_THRESH_1 = 0;
unsigned long counter_THRESH_END = 0;

unsigned long lastThreshTime = 0;
unsigned long bitsReceived = 0;


bool hasRunThresh = 0;
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("r")) {
      outputThresholds = !outputThresholds;
      Serial.print("Output mode: ");
      Serial.println(outputThresholds ? "Thresholds" : "Raw FFT data");
    } else if (input.equalsIgnoreCase("z")) {
      // zoomMode = !zoomMode;
      // if (zoomMode) {
      //   Serial.println("Zoom mode: ON (fast processing, 1Hz report)");
      //   counter_START = counter_0 = counter_1 = counter_END = 0;
      //   lastReportTime = millis();
      // } else {
      //   Serial.println("Zoom mode: OFF (normal output resumed)");
      // }
      Serial.println("Reset bit counter");
      bitsReceived = 0;
    } else if (input.equalsIgnoreCase("d") || !hasRunThresh) {
      hasRunThresh = 1;
      Serial.println("Sampling bandpass filter amplitudes for 10 seconds...");

      const unsigned long sampleDurationMs = 10000;
      const unsigned long startTime = millis();

      unsigned long totalStart = 0, total0 = 0, total1 = 0, totalEnd = 0;
      unsigned long samplesStart = 0, samples0 = 0, samples1 = 0, samplesEnd = 0;

      // Capture samples
      while (millis() - startTime < sampleDurationMs) {
        totalStart += sumFilterAmplitudes(branchF_START, &samplesStart);
        total0 += sumFilterAmplitudes(branchF_0, &samples0);
        total1 += sumFilterAmplitudes(branchF_1, &samples1);
        totalEnd += sumFilterAmplitudes(branchF_END, &samplesEnd);

        delay(2); // match main loop delay
      }

      // Compute average amplitudes
      double avgStart = (samplesStart > 0) ? (double)totalStart / (double)samplesStart : 0;
      double avg0 = (samples0 > 0) ? (double)total0 / (double)samples0 : 0;
      double avg1 = (samples1 > 0) ? (double)total1 / (double)samples1 : 0;
      double avgEnd = (samplesEnd > 0) ? totalEnd / (double)samplesEnd : 0;

      // Set thresholds
      numThresholds = 4;
      ampThresholds[0] = {MESSAGE_0_FREQ,     avg0  * 2};
      ampThresholds[1] = {MESSAGE_START_FREQ, avgStart * 2};
      ampThresholds[2] = {MESSAGE_1_FREQ,     avg1  * 2};
      ampThresholds[3] = {MESSAGE_END_FREQ,   avgEnd  * 2};

      // Sort thresholds
      for (int i = 0; i < numThresholds - 1; i++) {
        for (int j = i + 1; j < numThresholds; j++) {
          if (ampThresholds[i].freq > ampThresholds[j].freq) {
            FreqThreshold temp = ampThresholds[i];
            ampThresholds[i] = ampThresholds[j];
            ampThresholds[j] = temp;
          }
        }
      }

      Serial.println("New thresholds set:");
      for (int i = 0; i < numThresholds; i++) {
        Serial.print(ampThresholds[i].freq);
        Serial.print(" Hz → ");
        Serial.println(ampThresholds[i].amp);
      }
      delay(1000);
} else if (input.startsWith("a ")) {
        float freq = 0, amp = 0;
        int space1 = input.indexOf(' ');
        int space2 = input.indexOf(' ', space1 + 1);
        if (space2 > 0) {
            freq = input.substring(space1 + 1, space2).toFloat();
            amp = input.substring(space2 + 1).toFloat();

            // Check if freq already exists, update if so
            bool updated = false;
            for (int i = 0; i < numThresholds; i++) {
                if (abs(ampThresholds[i].freq - freq) < 1.0) {
                    ampThresholds[i].amp = amp;
                    updated = true;
                    break;
                }
            }

            // If not found, insert it
            if (!updated && numThresholds < MAX_THRESHOLDS) {
                ampThresholds[numThresholds++] = {freq, amp};
            }

            // Sort by frequency
            for (int i = 0; i < numThresholds - 1; i++) {
                for (int j = i + 1; j < numThresholds; j++) {
                    if (ampThresholds[i].freq > ampThresholds[j].freq) {
                        FreqThreshold temp = ampThresholds[i];
                        ampThresholds[i] = ampThresholds[j];
                        ampThresholds[j] = temp;
                    }
                }
            }

            Serial.print("Updated threshold for ");
            Serial.print(freq);
            Serial.print(" Hz → ");
            Serial.println(amp);
        } else {
            Serial.println("Invalid format. Use: a <freq> <amp>");
        }
    }
  }

  // Read average sample amplitudes in each filter
  double startAmp = getQueueAverageAmplitude(branchF_START->queue, zoomMode ? &counter_START : nullptr);
  double f0Amp    = getQueueAverageAmplitude(branchF_0->queue,    zoomMode ? &counter_0    : nullptr);
  double f1Amp    = getQueueAverageAmplitude(branchF_1->queue,    zoomMode ? &counter_1    : nullptr);
  double endAmp   = getQueueAverageAmplitude(branchF_END->queue,  zoomMode ? &counter_END  : nullptr);
  // if (!zoomMode) delay(1);
  delay(2);

  if (zoomMode) {
    if (millis() - lastReportTime >= 1000) {
        Serial.print("Filter sample rates (Hz):\t");
        Serial.print("Start=");
        Serial.print(counter_START);
        Serial.print("\t0=");
        Serial.print(counter_0);
        Serial.print("\t1=");
        Serial.print(counter_1);
        Serial.print("\tEnd=");
        Serial.println(counter_END);

        counter_START = counter_0 = counter_1 = counter_END = 0;
        lastReportTime = millis();
    }
  } else {

    if (millis() - lastThreshTime >= BIT_DURATION) {
      // 1/3 of buffer needed to trigger bits received
      if (counter_THRESH_START*2 >= counter_START) bitsReceived++;
      if (counter_THRESH_0*2 >= counter_0) bitsReceived++;
      if (counter_THRESH_1*2 >= counter_1) bitsReceived++;
      if (counter_THRESH_END*2 >= counter_END) bitsReceived++;

      // Serial.print(counter_THRESH_START);
      // Serial.print(" ");
      // Serial.println(counter_START);

      counter_THRESH_START = 0;
      counter_START = 0;
      counter_THRESH_0 = 0;
      counter_0 = 0;
      counter_THRESH_1 = 0;
      counter_1 = 0;
      counter_THRESH_END = 0;
      counter_END = 0;
      lastThreshTime = millis();
    }

    if (millis() - lastReportTime >= 1000) {
      Serial.print("[BITREPORT] Valid bits received: ");
      Serial.println(bitsReceived);
      lastReportTime = millis();
    }

    counter_THRESH_START += (startAmp >= getAmplitudeThreshold(MESSAGE_START_FREQ)) ? 1 : 0;
    counter_START++;

    counter_THRESH_0 += (f0Amp >= getAmplitudeThreshold(MESSAGE_0_FREQ)) ? 1 : 0;
    counter_0++;

    counter_THRESH_1 += (f1Amp >= getAmplitudeThreshold(MESSAGE_1_FREQ)) ? 1 : 0;
    counter_1++;

    counter_THRESH_END += (endAmp >= getAmplitudeThreshold(MESSAGE_END_FREQ)) ? 1 : 0;
    counter_END++;

  //   // Plot results
  //   if (outputThresholds) {

  //     Serial.print(startAmp >= getAmplitudeThreshold(MESSAGE_START_FREQ));
  //     Serial.print("\t");
  //     Serial.print(f0Amp >= getAmplitudeThreshold(MESSAGE_0_FREQ));
  //     Serial.print("\t");
  //     Serial.print(f1Amp >= getAmplitudeThreshold(MESSAGE_1_FREQ));
  //     Serial.print("\t");
  //     Serial.println(endAmp >= getAmplitudeThreshold(MESSAGE_END_FREQ));
  //   } else {
  //     Serial.print(startAmp);
  //     Serial.print("\t");
  //     Serial.print(f0Amp);
  //     Serial.print("\t");
  //     Serial.print(f1Amp);
  //     Serial.print("\t");
  
  //     Serial.println(endAmp);
  //   }
  }

  // delay(50); // Smooth plot
}
