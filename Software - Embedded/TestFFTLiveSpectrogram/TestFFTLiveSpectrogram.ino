/*******
 * TestFFTLiveSpectrogram
 * Will output peak FFT measured frequency, along with its (scaled) amplitude. Open serial plotter, then type 's' into console to enable output.
 * Author: Aaron Becker
 * 11 April 2025
 * For 6.1820
 */

// Import default libraries
#include <Arduino.h>

//Import audio-related libraries
#include <Audio.h>
#include "pindefs.h" // Pin definitions


/************* AUDIO SHIELD / PIPELINE SETUP */
const int micInput = AUDIO_INPUT_MIC;
const int chipSelect = 10; 

// SELECT SAMPLE RATE
const uint32_t sampleRate = 44100;
// const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

/*************** AUDIO INPUT CHAIN (HYDROPHONE IN) */
const int myInput = AUDIO_INPUT_MIC;
AudioControlSGTL5000    audioShield;
AudioInputI2S            audioInput;           //xy=180,111
AudioAmplifier           inputAmp;           //xy=470,93
AudioAnalyzeFFT1024      inputFFT;      //xy=616,102
AudioConnection          patchCord3(audioInput, 0, inputAmp, 0);
AudioConnection          patchCord4(inputAmp, 0, inputFFT, 0);

/************** SAMPLE BUFFER LOGIC / RECEIVING STATE MACHINE
Each bin is numbered 0-1023 and has a float with its amplitude
SamplingBuffer is a time-valued array that records bin number
*/
#define FFT_BIN_WIDTH 43.0664
#define MIN_VALID_AMP 0.05
#define MAX_VALID_AMP 1.5
#define MESSAGE_LOWPASS_CUTOFF_FREQ 4500
#define FFT_BIN_CUTOFF (int)(MESSAGE_LOWPASS_CUTOFF_FREQ/FFT_BIN_WIDTH) //Lowpass 
#define BIT_DELAY 500
unsigned long lastBitDelay = 0;
int bitsReceived = 0;
unsigned long lastPrintTime = 0;
float lastPeakFreq = 0;

void setup() {
    Serial.begin(115200);

    // Setup pin modes
    pinMode(LED_PIN, OUTPUT);
    pinMode(HYDROPHONE_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    // Initial pin setup
    digitalWrite(RELAY_PIN, LOW);

    // Get audio shield up and running
    AudioMemory(500);
    inputAmp.gain(3);        // amplify mic to useful range
    audioShield.enable();
    audioShield.inputSelect(myInput);
    audioShield.micGain(90);
    audioShield.volume(1);
    setAudioSampleI2SFreq(sampleRate); // Set I2S sampling frequency
    //Serial.printf("[OK] SGTL5000 initialized and running at samplerate: %d\n", sampleRate);

    // Prime da plotter
    Serial.println("Amplitude (0-1):\tFrequency (1=100kHz):");
}


unsigned int totalSampleCount = 0;
unsigned int validSampleCount = 0;

void loop() {
    static bool printResults = true;


    // Check for user input to start
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            printResults = !printResults;
        } else if (c == 'z' || c == 'Z') {
          bitsReceived = 0;
        }
    }

    if (inputFFT.available()) {
      float maxBinAmp = 0;
      int binNumber = 0;
      for (int i = 0; i < 1024; i++) {
          float n = inputFFT.read(i);
          if (n > maxBinAmp && i >= FFT_BIN_CUTOFF) {
              maxBinAmp = n;
              binNumber = i;
          }
      }

      // if (maxBinAmp >= MIN_VALID_AMP && maxBinAmp <= MAX_VALID_AMP) {
      float peakFreq = binNumber * FFT_BIN_WIDTH;
      float scaledFreq = peakFreq / 20000.0; // Scale frequency to 0-1 range, 100kHz -> 1. So 20kHz = 0.2

      // Print for Serial Plotter: tab-separated
      // Serial.print(maxBinAmp*5);
      // Serial.print("\t");
      // Serial.println(scaledFreq);
      
      // human-readable output
      totalSampleCount++;
      if (maxBinAmp > 0.075) {
        bool isZero = abs(peakFreq - 9000) <= 250;
        bool isOne = abs(peakFreq - 10000) <= 250;
        bool isStart = abs(peakFreq - 11000) <= 250;
        bool isEnd = abs(peakFreq - 12000) <= 250;
        if (isZero || isOne || isStart || isEnd && peakFreq != lastPeakFreq) {
          validSampleCount++;
          lastPeakFreq = peakFreq;
        }
      }
      
      if (printResults) {
        Serial.print(maxBinAmp); Serial.print(" @ "); Serial.print(peakFreq); Serial.println("Hz");
      }
    }

    if (millis() - lastBitDelay >= BIT_DELAY) {
      if (validSampleCount*2 >= totalSampleCount) bitsReceived++;
      validSampleCount = 0;
      totalSampleCount = 0;
      lastBitDelay = millis();
    }

    // if (millis() - lastPrintTime > 1000) {
    //   Serial.print("Bits received: ");
    //   Serial.println(bitsReceived);
    //   lastPrintTime = millis();
    // }
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