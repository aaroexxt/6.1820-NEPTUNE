/*******
 * NEPTUNE-NODE
 * Master script: sending, recieving, the whole works.
 * Author: Aaron Becker
 * 11 April 2025
 * For 6.1820
 */


// Import default libraries
#include <Arduino.h>

//Import audio-related libraries
#include <Audio.h>
// Import other device libraries
#include "Adafruit_MAX1704X.h" // Battery monitor
#include <Adafruit_BNO055.h> // Accel, Gyro, Mag, Temp
#include <Adafruit_NeoPixel.h> // LEDs

// Import all custom libraries
#include "pindefs.h" // Pin definitions
#include "UnderwaterMessage.h"
#include "Protocol.h"
#include "FreqThreshold.h"
#include "BandpassBranch.h"

/******* ADDITIONAL DEVICE SETUP */

// Declare NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// GRB color order
uint32_t red = strip.Color(255, 0, 0, 0);
uint32_t greenishwhite = strip.Color(0, 255, 0); // g r b w
uint32_t bluishwhite = strip.Color(0, 0, 255, 0);
uint32_t cyan = strip.Color(0, 255, 255, 0);
uint32_t purple = strip.Color(255, 0, 255, 0);

// LED blinky anim stuff
int counter = 0;
bool dir = 1;
unsigned long lastLEDUpdateTime = 0;

Adafruit_MAX17048 maxlipo; // Battery monitor
// Battery monitor stuff
float lastCellVoltage = 0.0;
float lastPercentage = 100.0; // Initialize with a full battery percentage
unsigned long lastBattCheckTime = 0;
unsigned long lastPlayedBatteryTime = 0;
int pctBufPointer = -1;
#define BATTERY_SAMPLES 5
float pctBuf[BATTERY_SAMPLES];

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t bno_orientationData, bno_angVelocityData, bno_magnetometerData, bno_accelerometerData;
int8_t bno_tempData;
unsigned long lastBNOReadTime = 0;
#define BNO055_SAMPLERATE_DELAY_MS 100

/************* AUDIO SHIELD / PIPELINE SETUP */
// For SGTL5000
const int micInput = AUDIO_INPUT_MIC;
const int chipSelect = 10;

/*************** AUDIO INPUT CHAIN (HYDROPHONE IN) */
AudioControlSGTL5000    audioShield;
AudioInputI2S            audioInput;           //xy=180,111
AudioAmplifier           inputAmp;           //xy=470,93
AudioConnection          patchCord3(audioInput, 0, inputAmp, 0);

/************** SAMPLE BUFFER LOGIC / RECEIVING STATE MACHINE
We use 4 bandpass filters around each frequency of interest
Each bandpass filter is a biquad filter with multiple cascaded stages to narrow the filter effective passband.
*/
int16_t samplingBuffer[NUM_SAMPLES]; // BIN indices
uint16_t samplingPointer = 0; //How many samples have we seen?
#define MESSAGE_LENGTH UnderwaterMessage::size
bool bitBuffer[MESSAGE_LENGTH]; // Message sample buffer (1 or 0)
int bitPointer = 0;
int bitsReceived = 0; // Trackers for bits and bit errors received (for tx/rx statistics)
int bitsTransmitted = 0;
int errorsReceived = 0;
unsigned long lastBitStatisticsTime = 0;
#define BANDWIDTH_FREQ 150 // Width of bounds around center frequency for each bandpass filter

// Instantiate bandpass filters for each branch
BandpassBranch* branchF_START;
BandpassBranch* branchF_0;
BandpassBranch* branchF_1;
BandpassBranch* branchF_END;

/**************** STATE MACHINE */

typedef enum {
  LISTENING, //Waiting for start frequency
  CHECK_START,
  INTERMEDIATE_START,
  DECODE_MESSAGE,
  MESSAGE_GET_BIT //Currently receiving bytes
} RECEIVING_STATE;

RECEIVING_STATE curReceivingState = LISTENING;
bool sampling = 0; // Are we currently sampling?

/*************** TRANSMITTING LOGIC (QUEUE) */
const byte toneBufferLength = 10*MESSAGE_LENGTH; //Arbitrarily can hold 10 messages at a time
int toneFreqQueue[toneBufferLength];
unsigned long toneDelayQueue[toneBufferLength];
unsigned long lastToneStart = 0;
byte toneStackPos = 0;
unsigned long lastBitChange = 0;

/*************** OPERATING STATE MACHINE */
typedef enum {
  RECEIVE,
  TRANSMIT,
  ERROR
} OPERATING_MODE;

OPERATING_MODE mode = RECEIVE;

/****************** FUNCTION PROTOTYPES */
// Forward declarations to help Arduino compiler
bool debugMode = false;
void transitionReceivingState(RECEIVING_STATE newState);
void transitionOperatingMode(OPERATING_MODE newMode);
int countTonePresentSamples(AudioRecordQueue* queue);
void clearSampleBuffer();
void clearBitBuffer();
bool isSampleBufferValid();
void transmitMessageAsync(UnderwaterMessage message);
void addToneQueue(int freq, unsigned long delay);

void setup() {
    Serial.begin(115200);
    
    AudioMemory(500);

    /******** INITIALIZATION */
    /*
    Steps:
    1) Initialize:
        - Blink blue to show that we are alive!
        - LED strip, battery monitor, audio shield
        - Blink green to show initialization good!
        - If anything fails: display full red on LED strip
    */
    int step = 1; // Initialization step

    // Setup pin modes
    pinMode(LED_PIN, OUTPUT);
    pinMode(HYDROPHONE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    // Initial pin setup
    digitalWrite(RELAY_PIN, LOW);
   
    // Get LEDs up and running
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(LED_BRIGHTNESS);

    // LEDs: show that we are alive
    strip.fill(purple, 0, 20); // light up entire strip
    strip.show();
    // keep strip on, then continue with rest of installation
    delay(500); 
    strip.clear();

    Serial.printf("[OK] NEPTUNE-NODE Initializing. Node-ID=%d\n", NODE_ID);

    initializationPass(step);
    step++;
    
    // Battery check setup
    if (!maxlipo.begin()) {
      Serial.println("[Error] Battery monitor");
      //initializationError(4);
    } else {
      Serial.print(F("[OK] Battery monitor MAX17048 initialized"));
      Serial.print(F(" with Chip ID: 0x")); 
      Serial.println(maxlipo.getChipID(), HEX);
      maxlipo.setAlertVoltages(2.0, 4.2);
    }
    initializationPass(step);
    step++;

    // Setup BNO055
    /* Initialise the sensor */
    if(!bno.begin())
    {
      Serial.print("[ERROR] Error intializing BNO055.");
      initializationError(step);
    }
  
    /* Use external crystal for better accuracy */
    bno.setExtCrystalUse(true);
    Serial.println("[OK] BNO055 initialized");
    initializationPass(step);
    step++;

    // Setup bandpass filters
    branchF_START = createBandpassBranch(inputAmp, sampleRate, MESSAGE_START_FREQ, BANDWIDTH_FREQ);
    branchF_0 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_0_FREQ, BANDWIDTH_FREQ);
    branchF_1 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_1_FREQ, BANDWIDTH_FREQ);
    branchF_END = createBandpassBranch(inputAmp, sampleRate, MESSAGE_END_FREQ, BANDWIDTH_FREQ);
    Serial.println("Bandpass filters initialization OK");
    initializationPass(step);
    step++;

    // Get audio shield up and running
    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(90);
    audioShield.volume(1);
    inputAmp.gain(2);        // amplify mic to useful range
    //setAudioSampleI2SFreq(sampleRate); // Set I2S sampling frequency
    Serial.printf("[OK] SGTL running at samplerate: %d\n", sampleRate);
    initializationPass(step);
    step++;

    // Setup receiver state machine, and transition states
    transitionReceivingState(LISTENING);
    transitionOperatingMode(RECEIVE);
    Serial.println("[OK] State machine initialized");
    initializationPass(step);
    step++;

    Serial.println("Done initializing!! Starting now! In receiving default mode!");
    Serial.println("Welcome to NEPTUNE >:)");
    strip.fill(bluishwhite, 0, 12); // light up entire strip, all set up!
    strip.show();

    // Get frequency bin thresholds
    measureFrequencyBinThresholds(FREQ_MEASURE_DURATION_MS, THRESHOLD_MULT_FACTOR);
}

unsigned long lastLoopTime = 0;

void loop() {
  /*********** LEDS */
  if (millis() > lastLEDUpdateTime && false) {
    uint32_t color = strip.Color(counter, 0, 0, 30); // r g b w
    if (dir) {
      counter++;
    } else {
      counter--;
    }
    if (counter <= 0 || counter >= 255) dir = !dir;
    strip.fill(color);
    strip.show();
    lastLEDUpdateTime = millis() + 1;
  }

  /****************** BATTERY MONITOR */
  if (millis() - lastBattCheckTime >= 1000 && false) { // Check every 1 second
    lastBattCheckTime = millis();

    float cellVoltage = maxlipo.cellVoltage(); // Get cell voltage
    if (isnan(cellVoltage)) {
      Serial.println("[BATT] Failed to read cell voltage, check battery is connected!");
    } else {
      float cellPercent = maxlipo.cellPercent(); // Get battery percentage
      Serial.printf("[BATT] Battery V: %d, %%:%d", cellVoltage, cellPercent);
      lastCellVoltage = cellVoltage;

      if (pctBufPointer == -1) { //First sample
        for (int i=0; i<BATTERY_SAMPLES; i++) {
          pctBuf[i] = maxlipo.cellVoltage();
        }
        pctBufPointer = 1; //Increment pointer
      } else {
        pctBuf[pctBufPointer] = maxlipo.cellVoltage();
      }
      pctBufPointer++;
      if (pctBufPointer >= BATTERY_SAMPLES) {
        pctBufPointer = 0;
      }

      float totPctBuf = 0.0;
      for (int i = 0; i < BATTERY_SAMPLES; i++) {
        totPctBuf += pctBuf[i];
      }

      float avgPctBuf = totPctBuf / 5.0;
      float bufDifference = maxlipo.cellVoltage() - avgPctBuf;

      if (millis() - lastPlayedBatteryTime > 3100) {
        if (bufDifference > 0.05) {
          Serial.println("[BATT] Neptune charging activated!");
        } else if (bufDifference < -0.05) {
          Serial.println("[BATT] Neptune charging deactivated!");
        }
        lastPlayedBatteryTime = millis();
      }

      // Check if the battery percentage has dropped by at least 10%
      if ((int(cellPercent) / 10) < (int(lastPercentage) / 10)) {
        int percentRange = (int(cellPercent) / 10) * 10;
        Serial.print("[BATT] Battery dropped to ");
        Serial.print(percentRange);
        Serial.println("% range.");

        lastPercentage = cellPercent; // Update the last known percentage
      }
    }
  }

  /******************* SERIAL COMMAND HANDLER */
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    // a command -> set amplitude threshold
    if (input.startsWith("a ")) {
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
    } else if (input.startsWith("m ")) {
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);
      if (space2 > 0) {
        unsigned long sampleDurationMs = (unsigned long)input.substring(space1 + 1, space2).toInt();
        float thresholdMultFactor = input.substring(space2 + 1).toFloat();
        
        // Pass to helper fn
        measureFrequencyBinThresholds(sampleDurationMs, thresholdMultFactor);
      } else {
        Serial.println("Invalid format. Use: m <sampleDurationMs> <thresholdMultFactor>");
      }
    } else if (input.startsWith("c ")) {
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);
      if (space2 > 0) {
        uint16_t id = (uint16_t)input.substring(space1 + 1, space2).toInt();
        uint16_t msg = (uint16_t)input.substring(space2 + 1).toFloat();
        
        UnderwaterMessage um(id, msg);
        uint16_t encoded = um.encodeHamming();

        // copy sim message bits into bitbuffer
        for (int i = 0; i < MESSAGE_LENGTH; i++) {
          bitBuffer[i] = (encoded >> i) & 0x01;
        }
        
        // Switch to decoding state
        transitionReceivingState(DECODE_MESSAGE);
      } else {
        Serial.println("Invalid format. Use c <simulatedID> <simulatedMsg>");
      }
    } else if (input.equalsIgnoreCase("d")) {
      debugMode = !debugMode;
      Serial.print("DebugMode State: ");
      Serial.println(debugMode ? "on" : "off");
    } else {
      Serial.println("NEPTUNE CLI Help\n--------------\nThreshold Setting: a <frequency> <amplitude>\nRemeasure Thresholds: m <sampleDurationMs> <thresholdMultFactor>\nSimulate Message: c <simID> <simMSG>\nToggle Debug Mode: d\n--------------");
    }
  }

  /************ IMU SENSOR */
  if (millis()-lastBNOReadTime >= BNO055_SAMPLERATE_DELAY_MS) {
    bno.getEvent(&bno_orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&bno_angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&bno_magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&bno_accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno_tempData = bno.getTemp();
    lastBNOReadTime = millis();
  }

  /**************** BIT STATISTICS */
  if (millis() - lastBitStatisticsTime >= 1000) {
    Serial.printf("[BIT STATISTICS] Tx: %d, Rx: %d, ErrRx: %d, %%ErrRx:%.3f\n", bitsTransmitted, bitsReceived, errorsReceived, (float)errorsReceived/(float)bitsReceived);
    lastBitStatisticsTime = millis();

    if (curReceivingState == LISTENING) clearSampleBuffer(); // TODO note this line
  }
  
  /**************** TONE SENDING */
  if (toneStackPos == 0) {
    strip.clear(); // Set all pixel colors to 'off' if queue is empty
    transitionOperatingMode(RECEIVE); // Switch relays to receive mode
  }

  if (toneStackPos > 0) {
    if (millis() - lastToneStart > toneDelayQueue[0]) {
      // Serial.print("ToneQueue: ");
      for (int i=1; i<toneBufferLength; i++) { //Left shift all results by 1
          // Serial.print(toneFreqQueue[i]);
          // Serial.print("Hz@");
          // Serial.print(toneDelayQueue[i]);
          toneFreqQueue[i-1] = toneFreqQueue[i];
          toneDelayQueue[i-1] = toneDelayQueue[i];
      }
      // Serial.println();
      toneStackPos--; //we’ve removed one from the stack
      if (toneStackPos > 0) { //is there something new to start playing?
          if (toneFreqQueue[0] > 0) {
            tone(HYDROPHONE_PIN, toneFreqQueue[0]); //start new tone
            bitsTransmitted++;
          }
          lastToneStart = millis();
      } else {
          noTone(HYDROPHONE_PIN); //otherwise just stop playing
      }
    }
  }

  /**************** TONE RECEIVING */
  // Input samples into samplebuffer. CountTonePresentSamples will free queue memory as necessary
  // 2nd argument is threshold, input values are -32768 to 32767 so this is sort of arbitrary
  int startSamples = countTonePresentSamples(branchF_START);
  int F0Samples = countTonePresentSamples(branchF_0);
  int F1Samples = countTonePresentSamples(branchF_1);
  int endSamples = countTonePresentSamples(branchF_END);
  
  if (debugMode) {
    if (startSamples > 100) {Serial.print("S"); Serial.println(startSamples);}
    if (F0Samples > 100) {Serial.print("0"); Serial.println(F0Samples);}
    if (F1Samples > 100) {Serial.print("1"); Serial.println(F1Samples);}
    if (endSamples > 100) {Serial.print("E"); Serial.println(endSamples);}
  }

  while(millis() - lastLoopTime < 10) {} // Accumulate 10ms of samples between iters
  lastLoopTime = millis();

  // Then add samples to sampling buffer
  if (sampling) {
    addSamplesToBuffer(MESSAGE_START_FREQ, startSamples);
    addSamplesToBuffer(MESSAGE_0_FREQ, F0Samples);
    addSamplesToBuffer(MESSAGE_1_FREQ, F1Samples);
    addSamplesToBuffer(MESSAGE_END_FREQ, endSamples);
  }
  
  if (curReceivingState == LISTENING) {

    // Does the sampling buffer currently contain at least THRESHOLD_SAMPLES_MIN_DETECT examples of start frequency?
    if (doesSampleBufferHaveCountOfFreq(MESSAGE_START_FREQ, THRESHOLD_SAMPLES_MIN_DETECT)) {
      transitionReceivingState(CHECK_START);
    }

  } else if ((curReceivingState == CHECK_START || curReceivingState == MESSAGE_GET_BIT) && (doesSampleBufferHaveCountOfFreq(MESSAGE_END_FREQ, THRESHOLD_SAMPLES_MIN_DETECT) || bitPointer >= MESSAGE_LENGTH)) { // We got the end message bit OR bit buffer is full, thus message is over
    transitionReceivingState(DECODE_MESSAGE);

  } else if (curReceivingState == CHECK_START && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_0_FREQ, THRESHOLD_SAMPLES_MIN_DETECT) || doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_MIN_DETECT))) { // Gotten all start samples OR have we started to transition into the bit (ie our timing was misaligned)?

    if (doesSampleBufferHaveCountOfFreq(MESSAGE_START_FREQ, THRESHOLD_SAMPLES_MIN_DETECT)) {
      transitionReceivingState(MESSAGE_GET_BIT); //Currently receiving valid message
    } else { // If buffer is not valid OR freq doesn’t match start
      transitionReceivingState(LISTENING);
    }

  } else if (curReceivingState == INTERMEDIATE_START && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_0_FREQ, THRESHOLD_SAMPLES_MIN_DETECT) || doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_MIN_DETECT))) { //Either bitChange time has elapsed or we got a 0 or 1 frequency

    transitionReceivingState(MESSAGE_GET_BIT);

  } else if (curReceivingState == MESSAGE_GET_BIT && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_START_FREQ, THRESHOLD_SAMPLES_MIN_DETECT))) { // Gotten our 1/0 samples OR have we started to transition back to start state (ie timing misaligned again)
    
    if (doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_VALID)) {
      bitBuffer[bitPointer] = 1; // WE GOT A 1
    } else {
      bitBuffer[bitPointer] = 1; // WE GOT A 0
    }
    bitPointer++;
    bitsReceived++;
    transitionReceivingState(INTERMEDIATE_START);

  } else if (curReceivingState == DECODE_MESSAGE) {
    // Message decoding here
    uint16_t encoded = 0;
    // Combine bits in bitBuffer into a single byte
    for (int i = MESSAGE_LENGTH - 1; i >= 0; i--) {
      encoded = (encoded << 1) | bitBuffer[i];
    }

    int errorPos = 0;
    bool doubleError = false;
    UnderwaterMessage decoded = UnderwaterMessage::decodeHamming(encoded, &errorPos, &doubleError);
    UnderwaterMessage response(NODE_ID, 0);

    if (doubleError) {
        Serial.println("[RECV] Double-bit uncorrectable error in received result");
        errorsReceived+=2;
        response.msg = 255; // Double bit error - put 255 in message field
        transmitMessageAsync(response); // Send error response back to transmitter TODO maybe comment out
    } else if (errorPos > 0) {
        Serial.print("[RECV] Corrected single-bit error at position: ");
        Serial.println(errorPos);
        errorsReceived++;
    } else {
        Serial.println("[RECV] No error detected.");
    }

    // Bit of an edge case: the double error could have been in the ID field. But if that's true, the message is simply invalid. For now we will send back a 255 error
    if (!doubleError) {
      Serial.printf("[MSG] RX ID: |0x%.2x|, MSG: |0x%.8x|\n", decoded.getID(), decoded.getMsg());
      if (decoded.getID() == NODE_ID) { // Only respond to messages to our ID
        response.id = NODE_ID; // Response ID is our node id
        switch (decoded.getMsg()) {
          case 1: // Gyro X data
            response.msg = (int8_t)mapf(bno_angVelocityData.gyro.x, -200.0, 200.0, 0.0, 255.0);
            break;
          case 2: // Gyro Y data
            response.msg = (int8_t)mapf(bno_angVelocityData.gyro.y, -200.0, 200.0, 0.0, 255.0);
            break;
          case 3: // Gyro Z data
            response.msg = (int8_t)mapf(bno_angVelocityData.gyro.z, -200.0, 200.0, 0.0, 255.0);
            break;
          case 4: // Accel X data
            response.msg = (int8_t)mapf(bno_accelerometerData.acceleration.x, -5.0, 5.0, 0.0, 255.0);
            break;
          case 5: // Accel Y data
            response.msg = (int8_t)mapf(bno_accelerometerData.acceleration.y, -5.0, 5.0, 0.0, 255.0);
            break;
          case 6: // Accel Z data
            response.msg = (int8_t)mapf(bno_accelerometerData.acceleration.z, -5.0, 5.0, 0.0, 255.0);
            break;
          case 7: // Mag X data
            response.msg = (int8_t)mapf(bno_magnetometerData.magnetic.x, -500.0, 500.0, 0.0, 255.0);
            break;
          case 8: // Mag Y data
            response.msg = (int8_t)mapf(bno_magnetometerData.magnetic.y, -500.0, 500.0, 0.0, 255.0);
            break;
          case 9: // Mag Z data
            response.msg = (int8_t)mapf(bno_magnetometerData.magnetic.z, -500.0, 500.0, 0.0, 255.0);
            break;
          case 10: // Ori X data
            response.msg = (int8_t)mapf(bno_orientationData.orientation.x, 0.0, 360.0, 0.0, 255.0);
            break;
          case 11: // Ori Y data
            response.msg = (int8_t)mapf(bno_orientationData.orientation.y, 0.0, 360.0, 0.0, 255.0);
            break;
          case 12: // Ori Z data
            response.msg = (int8_t)mapf(bno_orientationData.orientation.z, 0.0, 360.0, 0.0, 255.0);
            break;
          case 13: // Temp data
            response.msg = (int8_t)bno_tempData; // Already int8
            break;

          case 14: // Toggle LEDs
            if (LED_BRIGHTNESS == 0) {
              LED_BRIGHTNESS = 100;
              response.msg = 0xFF; //return new state
            } else {
              LED_BRIGHTNESS = 0;
              response.msg = 0x00;
            }
            strip.setBrightness(LED_BRIGHTNESS);
            break;

          case 15: // Get battery voltage
            response.msg = (int8_t)mapf(lastCellVoltage, 0.0, 4.2, 0.0, 255.0);
            break;
          
          case 16: // Get battery percent
            response.msg = (int8_t)mapf(lastPercentage, 0.0, 100.0, 0.0, 255.0);
            break;
          
          case 0: // Alive check
          default:
            response.msg = 0xAA; // Alive check is alternating 0s and 1s
            break;
        }

        transmitMessageAsync(response); // Send response back to transmitter
      } else {
        Serial.printf("[RECV] Message not to our ID (was for ID=%d), not responding\n", decoded.getID());
      }
    }
    
    transitionReceivingState(LISTENING); // Go back to listening once decoding is done
  }
}

// Transition state function
// Handles all state-edge transitions
void transitionReceivingState(RECEIVING_STATE newState) {
  if (newState >= 2) {
    Serial.print("transitionReceivingState: ");
    Serial.println(newState);
  }
  if (newState == CHECK_START || newState == INTERMEDIATE_START || newState == MESSAGE_GET_BIT) {
    clearSampleBuffer();
    sampling = 1; // Begin sampling
    lastBitChange = millis(); // Start timer
  } else if (newState == DECODE_MESSAGE) {
    sampling = 0; // Stop sampling
  } else { //Back to LISTENING
    clearSampleBuffer();
    clearBitBuffer();
    sampling = 1;
  }
  curReceivingState = newState; // Set our current state to the new one
}

void measureFrequencyBinThresholds(unsigned long sampleDurationMs, float thresholdMultFactor) {
  // Init counters
  unsigned long totalStart = 0, total0 = 0, total1 = 0, totalEnd = 0;
  unsigned long samplesStart = 0, samples0 = 0, samples1 = 0, samplesEnd = 0;

  const unsigned long startTime = millis();

  // Capture samples
  while (millis() - startTime < sampleDurationMs) {
    totalStart += sumFilterAmplitudes(branchF_START, &samplesStart);
    total0 += sumFilterAmplitudes(branchF_0, &samples0);
    total1 += sumFilterAmplitudes(branchF_1, &samples1);
    totalEnd += sumFilterAmplitudes(branchF_END, &samplesEnd);

    delay(10); // match main loop delay
  }

  // Compute average amplitudes
  float avgStart = (samplesStart > 0) ? (float)totalStart / (float)samplesStart : 0;
  float avg0 = (samples0 > 0) ? (float)total0 / (float)samples0 : 0;
  float avg1 = (samples1 > 0) ? (float)total1 / (float)samples1 : 0;
  float avgEnd = (samplesEnd > 0) ? (float)totalEnd / (float)samplesEnd : 0;

  // Set thresholds
  numThresholds = 4;
  ampThresholds[0] = {MESSAGE_0_FREQ, avg0 * thresholdMultFactor};
  ampThresholds[1] = {MESSAGE_START_FREQ, avgStart * thresholdMultFactor};
  ampThresholds[2] = {MESSAGE_1_FREQ, avg1 * thresholdMultFactor};
  ampThresholds[3] = {MESSAGE_END_FREQ, avgEnd * thresholdMultFactor};

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
    Serial.print(" Hz -> ");
    Serial.println(ampThresholds[i].amp);
  }
}

void addSamplesToBuffer(int sampleFreq, int count) {
  //Serial.print(sampleFreq); Serial.print("Hz @ c="); Serial.println(count);
  for (int i=0; i<count; i++) {
    samplingBuffer[samplingPointer] = sampleFreq; // Commit sample (bin number) to memory
      samplingPointer++;
      if (samplingPointer >= NUM_SAMPLES) { // Wrap around end of buffer
        samplingPointer = 0;
      }
  }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool doesSampleBufferHaveCountOfFreq(int freq, int countMatch) {
  int count = 0;
  for (int i=0; i<NUM_SAMPLES; i++) {
    if (samplingBuffer[i] == freq) count++;
  }
  return count >= countMatch;
}

void initializationPass(int check) {
    strip.fill(greenishwhite, 0, check); // check = number of tiles lit up
    strip.show();
    delay(100);
    strip.clear();
}

void initializationError(int error) {
    strip.fill(red, 0, error); // error = number of tiles lit up
    strip.show();
    delay(2000);
    strip.clear();
    while(1){}
}

void transitionOperatingMode(OPERATING_MODE newMode) {
    if (newMode == RECEIVE) {
        digitalWrite(RELAY_PIN, LOW); // make relays go to listen mode
    } else if (newMode == TRANSMIT) {
        digitalWrite(RELAY_PIN, HIGH); // make relays go into transmit mode
    }
    mode = newMode;
}

void clearSampleBuffer() {
  // Clear sampling buffer
  for (int i=0; i<NUM_SAMPLES; i++) {
    samplingBuffer[i] = -1; // Invalid sample
  }
  //Reset sampling pointer
  samplingPointer = 0;
}

void clearBitBuffer() {
  // Clear bit (received message) buffer
  for (int i=0; i<MESSAGE_LENGTH; i++) {
    bitBuffer[i] = 0;
  }
  //Reset bit pointer
  bitPointer = 0;
}

// Check whether sample buffer is valid
bool isSampleBufferValid() {
  int validCount = 0;
  for (int i=0; i<NUM_SAMPLES; i++) {
    if (samplingBuffer[i] != -1) {
      validCount++;
    }
  }
  return (validCount >= ((NUM_SAMPLES)/3));
}

// Function to transmit the UnderwaterMessage asynchronously
void transmitMessageAsync(UnderwaterMessage message) {
  Serial.printf("[MSG] TX ID: |0x%.2x|, MSG: |0x%.8x|\n", message.getID(), message.getMsg());

  uint16_t encoded = message.encodeHamming(); //Use hamming encoding scheme

  transitionOperatingMode(TRANSMIT);
  addToneQueue(MESSAGE_START_FREQ, MESSAGE_BIT_DELAY);
  // Iterate over each bit of the message
  for (int i = 0; i < MESSAGE_LENGTH; i++) {
    // Extract the i-th bit from the message data (starting from LSB)
    if (encoded & (1 << i)) { // If the i-th bit is 1
      addToneQueue(MESSAGE_1_FREQ, MESSAGE_BIT_DELAY);
    } else { // If the i-th bit is 0
      addToneQueue(MESSAGE_0_FREQ, MESSAGE_BIT_DELAY);
    }
    addToneQueue(MESSAGE_START_FREQ, MESSAGE_BIT_DELAY);
  }
  addToneQueue(MESSAGE_END_FREQ, (int)(MESSAGE_BIT_DELAY)); // End transmission with a stop tone
}

void addToneQueue(int freq, unsigned long delay) {
  if (toneStackPos < toneBufferLength && delay > 0) {
    toneFreqQueue[toneStackPos] = freq;
    toneDelayQueue[toneStackPos] = delay;
    toneStackPos++; //always increase stack pointer
    if (toneStackPos == 1) { //If it’s the first sound, start playing it
      tone(HYDROPHONE_PIN, toneFreqQueue[0]); //start new tone
      lastToneStart = millis();
    }
  }
}


/**************** Support SGTL sampling frequency changing */
#if defined(__IMXRT1062__)  // Teensy 4.x

#include <utility/imxrt_hw.h>

// taken from: https://forum.pjrc.com/threads/57283-Change-sample-rate-for-Teensy-4-(vs-Teensy-3)?p=213007&viewfull=1#post213007
void setAudioSampleI2SFreq(int freq) {
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
       | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
       | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f
//Serial.printf("setAudioSampleI2SFreq(%d)\n",freq);
}

#elif defined(KINETISK)  // Teensy 3.x

// samplerate code by Frank Boesing
// https://forum.pjrc.com/threads/38753-Discussion-about-a-simple-way-to-change-the-sample-rate
void setAudioSampleI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } __attribute__((__packed__)) tmclk;
  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, static_cast<int>(44117.64706) , 48000, 88200, static_cast<int>(44117.64706 * 2), 96000, 176400, static_cast<int>(44117.64706 * 4), 192000};

// F_PLL = phase lock loop output frequency = teensy CPU clock speed in Hz

#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==180000000)
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

#endif