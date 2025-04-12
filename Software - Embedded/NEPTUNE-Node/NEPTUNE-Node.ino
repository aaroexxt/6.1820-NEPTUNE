// Import default libraries
#include <Arduino.h>
#include <elapsedMillis.h>
#include <cmath>
#include <iostream>

//Import audio-related libraries
#include <Audio.h>
#include <SPI.h>
#include <Wire.h>

// Import other device libraries
#include "Adafruit_MAX1704X.h" // Battery monitor
#include <Adafruit_BNO055.h> // Accel, Gyro, Mag, Temp
#include <Adafruit_NeoPixel.h> // LEDs

// Import default pindefs
#include "pindefs.h" // Pin definitions
#include "UnderwaterMessage.h"

/******* ADDITIONAL DEVICE SETUP */

// Declare NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800);
// GRB color order
uint32_t red = strip.Color(0, 64, 0, 0);
uint32_t greenishwhite = strip.Color(64, 0, 0, 64); // g r b w
uint32_t bluishwhite = strip.Color(0, 0, 64, 64);

Adafruit_LC709203F lc;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t bno_orientationData, bno_angVelocityData, bno_magnetometerData, bno_accelerometerData;
int8_t bno_tempData;
unsigned long lastBNOReadTime = 0;
#define BNO055_SAMPLERATE_DELAY_MS 100

/************* AUDIO SHIELD / PIPELINE SETUP */
// For SGTL5000
const int micInput = AUDIO_INPUT_MIC;
const int chipSelect = 10;

// SELECT SAMPLE RATE
const uint32_t sampleRate = 44100;
//const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

/*************** AUDIO INPUT CHAIN (HYDROPHONE IN) */
AudioControlSGTL5000    audioShield;
AudioInputI2S            audioInput;           //xy=180,111
AudioAmplifier           inputAmp;           //xy=470,93
AudioConnection          patchCord3(audioInput, 0, inputAmp, 0);

/************** SAMPLE BUFFER LOGIC / RECEIVING STATE MACHINE
We use 4 bandpass filters around each frequency of interest
Each bandpass filter is a biquad filter with multiple cascaded stages to narrow the filter effective passband.
*/
#define MESSAGE_BIT_DELAY 250 // ms between bits
// in theory, we are getting 44100 samples out of bandpass per sec
#define NUM_SAMPLES (int)((MESSAGE_BIT_DELAY / 1000.0) * sampleRate)
int16_t samplingBuffer[NUM_SAMPLES]; // BIN indices
uint16_t samplingPointer = 0; //How many samples have we seen?
#define MESSAGE_LENGTH UnderwaterMessage::size
bool bitBuffer[MESSAGE_LENGTH]; // Message sample buffer (1 or 0)
int bitPointer = 0;
int bitsReceived = 0; // Trackers for bits and bit errors received (for tx/rx statistics)
int bitsTransmitted = 0;
int errorsReceived = 0;
unsigned long lastBitStatisticsTime = 0;
#define MESSAGE_START_FREQ 20000 // Hz (1/sec)
#define MESSAGE_0_FREQ 17500 // Hz
#define MESSAGE_1_FREQ 15000 // Hz
#define MESSAGE_END_FREQ 22000 // Hz (1/sec)

#define MIN_VALID_AMP 0.1
#define MAX_VALID_AMP 1.5
#define BANDWIDTH_FREQ 500 // Width of bounds around center frequency for each bandpass filter

/********************* BANDPASS FILTERS */
struct BandpassBranch {
  // Keep track of the biquad filter (multi-stage used), the audio queue and audio connection objects
  AudioFilterBiquad* filter;
  AudioRecordQueue* queue;
  AudioConnection* inputToFilter;
  AudioConnection* filterToQueue;
};

BandpassBranch* createBandpassBranch(AudioStream& input,
                                     float sampleRate,
                                     float centerFreq,
                                     float bandwidth,
                                     int stages = 3) {
  // Allocate a container for all parts of the branch
  BandpassBranch* branch = new BandpassBranch;

  // Allocate audio components
  branch->filter = new AudioFilterBiquad();
  branch->queue = new AudioRecordQueue();

  // Compute quality factor for each stage
  float Q_total = centerFreq / bandwidth;
  float Q_stage = Q_total / sqrt((float)stages);
  float coeffs[5];

  for (int i = 0; i < stages; i++) {
    AudioFilterBiquad::makeBandpass(coeffs, sampleRate, centerFreq, Q_stage);
    branch->filter->setCoefficients(i, coeffs);
  }

  // Hook up audio connections
  branch->inputToFilter = new AudioConnection(input, 0, *branch->filter, 0);
  branch->filterToQueue = new AudioConnection(*branch->filter, 0, *branch->queue, 0);

  return branch;
}

// Create bandpass filters for each branch
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


typedef enum {
  RECEIVE,
  TRANSMIT,
  ERROR
} OPERATING_MODE;

OPERATING_MODE mode = RECEIVE;

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

    // Setup pin modes
    pinMode(LED_PIN, OUTPUT);
    pinMode(HYDROPHONE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
   
    // Get LEDs up and running
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(64);

    // LEDs: show that we are alive
    strip.fill(bluishwhite, 0, 12); // light up entire strip
    strip.show();
    // keep strip on, then continue with rest of installation
    delay(500); 
    strip.clear();

    Serial.printf("NEPTUNE-NODE Initializing. Node-ID=%d\n", NODE_ID);

    initializationPass(2);
    
    // Get battery monitor (LC709203F) up and running
    Wire.setClock(100000);
    // Battery check setup
    if (!lc.begin()) {
      Serial.println(F("Error initializing LC709203F?\nMake sure a battery is plugged in!"));
      initializationError(6);
      while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    lc.setThermistorB(3950);
    Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_1000MAH);
    lc.setAlarmVoltage(3.8);

    Serial.println("Printing battery info:");
    printBatteryData();

    // Setup BNO055
    /* Initialise the sensor */
    if(!bno.begin())
    {
      Serial.print("Error intializing BNO055 ... Check your wiring or I2C ADDR!");
      while(1);
    }
  
    /* Use external crystal for better accuracy */
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialization OK");

    // Setup bandpass filters
    branchF_START = createBandpassBranch(inputAmp, sampleRate, MESSAGE_START_FREQ, BANDWIDTH_FREQ);
    branchF_0 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_0_FREQ, BANDWIDTH_FREQ);
    branchF_1 = createBandpassBranch(inputAmp, sampleRate, MESSAGE_1_FREQ, BANDWIDTH_FREQ);
    branchF_END = createBandpassBranch(inputAmp, sampleRate, MESSAGE_END_FREQ, BANDWIDTH_FREQ);
    Serial.println("Bandpass filters initialization OK");
    initializationPass(3);

    // Get audio shield up and running
    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(90);
    audioShield.volume(1);
    inputAmp.gain(2);        // amplify mic to useful range
    setAudioSampleI2SFreq(sampleRate); // Set I2S sampling frequency
    Serial.printf("SGTL running at samplerate: %d\n", sampleRate);
    initializationPass(4);

    // Setup receiver state machine, and transition states
    transitionReceivingState(LISTENING);
    transitionOperatingMode(RECEIVE);
    initializationPass(6);

    Serial.println("Done initializing! Starting now! In receiving default mode!");
    strip.fill(bluishwhite, 0, 12); // light up entire strip, all set up!
    strip.show();
    delay(100);
}

// LED blinky anim stuff
int counter = 0;
bool dir = 1;
long lastLEDUpdateTime = 0;
long lastButtonCheckTime = 0;

void loop() {
  /*********** LEDS */
  if (millis() > lastLEDUpdateTime) {
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

  /************ IMU SENSOR */
  if (millis()-lastBNOReadTime >= BNO055_SAMPLERATE_DELAY_MS) {
    bno.getEvent(&bno_orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&bno_angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno_tempData = bno.getTemp();
    lastBNOReadTime = millis();
  }

  /**************** BIT STATISTICS */
  if (millis() - lastBitStatisticsTime >= 1000) {
    Serial.printf("[BIT STATISTICS] Tx: %d, Rx: %d, ErrRx: %d, \%ErrRx:%.3f\n", bitsTransmitted, bitsReceived, errorsReceived, (float)errorsReceived/(float)bitsReceived);
    lastBitStatisticsTime = millis();
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
  startSamples = countTonePresentSamples(branchF_START, 2000);
  F0Samples = countTonePresentSamples(branchF_0, 2000);
  F1Samples = countTonePresentSamples(branchF_1, 2000);
  endSamples = countTonePresentSamples(branchF_END, 2000);

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
    transitionState(DECODE_MESSAGE);

  } else if (curRecievingState == CHECK_START && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_0_FREQ, THRESHOLD_SAMPLES_MIN_DETECT) || doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_MIN_DETECT))) { // Gotten all start samples OR have we started to transition into the bit (ie our timing was misaligned)?

    if (doesSampleBufferHaveCountOfFreq(MESSAGE_START_FREQ, THRESHOLD_SAMPLES_VALID)) {
      transitionReceivingState(MESSAGE_GET_BIT); //Currently receiving valid message
    } else { // If buffer is not valid OR freq doesn’t match start
      transitionReceivingState(LISTENING);
    }

  } else if (curReceivingState == INTERMEDIATE_START && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_0_FREQ, THRESHOLD_SAMPLES_MIN_DETECT) || doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_MIN_DETECT))) { //Either bitChange time has elapsed or we got a 0 or 1 frequency

    transitionReceivingState(MESSAGE_GET_BIT);

  } else if (curReceivingState == MESSAGE_GET_BIT && (millis() - lastBitChange >= MESSAGE_BIT_DELAY || doesSampleBufferHaveCountOfFreq(MESSAGE_START_FREQ, 2))) { // Gotten our 1/0 samples OR have we started to transition back to start state (ie timing misaligned again)
    
    if (doesSampleBufferHaveCountOfFreq(MESSAGE_1_FREQ, THRESHOLD_SAMPLES_VALID)) {
      bitBuffer[bitPointer] = 1; // WE GOT A 1
    } else {
      bitBuffer[bitPointer] = 1; // WE GOT A 0
    }
    bitPointer++;
    bitsReceived++;
    transitionState(INTERMEDIATE_START);

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
    UnderwaterMessage response;

    if (doubleError) {
        Serial.println("[RECV] Double-bit uncorrectable error in received result");
        errorsReceived+=2;
        response.id = 255; // Double bit error
        transmitMessageAsync(response); // Send error response back to transmitter TODO maybe comment out
    } else if (errorPos > 0) {
        Serial.print("[RECV] Corrected single-bit error at position: ");
        Serial.println(errorPos);
        errorsReceived++;
    } else {
        Serial.println("No error detected.");
    }

    // Bit of an edge case: the double error could have been in the ID field. But if that's true, the message is simply invalid. For now we will send back a 255 error
    if (!doubleError) {
      if (decoded.getID() == NODE_ID) { // Only respond to messages to our ID
        response.id = NODE_ID; // Response ID is our node id
        switch (decoded.getMsg()) {
          case 0: // Alive check
            response.msg = 0xFF; // All 1s
            break;
          case 1: // Gyro X data
            response.msg = (int8_t)mapf(bno_angVelocityData->gyro.x, -200.0, 200.0, 0.0, 255.0);
            break;
          case 2: // Gyro Y data
            response.msg = (int8_t)mapf(bno_angVelocityData->gyro.y, -200.0, 200.0, 0.0, 255.0);
            break;
          case 3: // Gyro Z data
            response.msg = (int8_t)mapf(bno_angVelocityData->gyro.z, -200.0, 200.0, 0.0, 255.0);
            break;
          case 4: // Accel X data
            response.msg = (int8_t)mapf(bno_accelerometerData->acceleration.x, -5.0, 5.0, 0.0, 255.0);
            break;
          case 5: // Accel Y data
            response.msg = (int8_t)mapf(bno_accelerometerData->acceleration.y, -5.0, 5.0, 0.0, 255.0);
            break;
          case 6: // Accel Z data
            response.msg = (int8_t)mapf(bno_accelerometerData->acceleration.z, -5.0, 5.0, 0.0, 255.0);
            break;
          case 7: // Mag X data
            response.msg = (int8_t)mapf(bno_magnetometerData->magnetic.x, -500.0, 500.0, 0.0, 255.0);
            break;
          case 8: // Mag Y data
            response.msg = (int8_t)mapf(bno_magnetometerData->magnetic.y, -500.0, 500.0, 0.0, 255.0);
            break;
          case 9: // Mag Z data
            response.msg = (int8_t)mapf(bno_magnetometerData->magnetic.z, -500.0, 500.0, 0.0, 255.0);
            break;
        }

        transmitMessageAsync(response); // Send response back to transmitter
      }
    }
    
    transitionState(LISTENING); // Go back to listening once decoding is done
  }
}

// Transition state function
void transitionReceivingState(RECEIVING_STATE newState) {
  Serial.print("transitionReceivingState: ");
  Serial.println(newState);
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

int countTonePresentSamples(AudioRecordQueue* queue, int16_t threshold) {
  int matches = 0;
  while (queue->available() > 0) {
    audio_block_t* block = queue->readBuffer();
    if (!block) continue;

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      if (abs(block->data[i]) >= threshold) matches++;
    }

    queue->freeBuffer();  // Free the block
  }

  return matches;  // No tone detected
}

void addSamplesToBuffer(int sampleFreq, int count) {
  for (int i=0; i<count; i++) {
    samplingBuffer[samplingPointer] = sampleFreq; // Commit sample (bin number) to memory
      samplingPointer++;
      if (samplingPointer >= NUM_SAMPLES) { // Wrap around end of buffer
        samplingPointer = 0;
      }
  }
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void doesSampleBufferHaveCountOfFreq(int freq, int countMatch) {
  int count = 0;
  for (int i=0; i<NUM_SAMPLES; i++) {
    if (sampleBuffer[i] == freq) count++;
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
}

void transitionOperatingMode(OPERATING_MODE newMode) {
    if (newMode == RECEIVE) {
        digitalWrite(RELAY_PIN, LOW); // make relays go to listen mode
    } else if (newMode == TRANSMIT) {
        digitalWrite(RELAY_PIN, HIGH); // make relays go into transmit mode
    }
    mode = newMode;
}

void printBatteryData(){
  //colors
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t red = strip.Color(255, 0, 0);
  uint32_t orange = strip.Color(255, 150, 0);
  uint32_t yellow = strip.Color(255, 255, 0);

    // time = millis()
    Serial.print("Batt_Voltage:");
    Serial.print(lc.cellVoltage(), 3);
    Serial.print("\t");
    Serial.print("Batt_Percent:");
    Serial.print(lc.cellPercent(), 1);
    Serial.print("\t");
    Serial.print("Batt_Temp:");
    Serial.println(lc.getCellTemperature(), 1);

    if (lc.cellPercent() > 80){
        strip.fill(green, 0, 7);
    }
    else if (lc.cellPercent() > 60){
        strip.fill(yellow, 0, 5);
    }
    else if (lc.cellPercent() > 20){
        strip.fill(orange, 0, 3);
    }
    else{
        strip.fill(red, 0, 1);
    }
    strip.show();
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