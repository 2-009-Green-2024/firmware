#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=99,60
AudioAnalyzeFFT1024      fft_pre;
AudioFilterBiquad        biquad1;        //xy=257,60
AudioAnalyzeFFT1024      fft_post;
AudioOutputI2S           i2s2;           //xy=416,60
AudioConnection          patchCord1(i2s1, 0, fft_pre, 0);
AudioConnection          patchCord2(i2s1, 0, biquad1, 0);
AudioConnection          patchCord3(biquad1, 0, fft_post, 0);
AudioConnection          patchCord4(biquad1, 0, i2s2, 0);
AudioConnection          patchCord5(biquad1, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=305,132
// GUItool: end automatically generated code


const int myInput = AUDIO_INPUT_LINEIN;
//const int myInput = AUDIO_INPUT_MIC;

void setup() {

  AudioMemory(500);
  sgtl5000_1.enable();  // Enable the audio shield
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.5);

  // Butterworth filter, 12 db/octave
  biquad1.setBandpass(0, 30000, 5000);

  // Linkwitz-Riley filter, 48 dB/octave
  // biquad1.setLowpass(0, 30000, 0.54);
  //biquad1.setLowpass(1, 800, 1.3);
  //biquad1.setLowpass(2, 800, 0.54);
  //biquad1.setLowpass(3, 800, 1.3);
}


void loop() {
  // if (fft_pre.available() && fft_post.available()) {
  //   // Print column headers
  //   Serial.println("Frequency,Pre-filter,Post-filter");
    
  //   // Print data for each frequency bin
  //   for (int i = 0; i < 512; i++) {
  //     float freq = i * (44100.0 / 1024.0); // Calculate frequency for this bin
  //     Serial.print(freq, 2); // Print frequency with 2 decimal places
  //     Serial.print(",");
  //     Serial.print(fft_pre.read(i), 6); // Print pre-filter magnitude with 6 decimal places
  //     Serial.print(",");
  //     Serial.println(fft_post.read(i), 6); // Print post-filter magnitude with 6 decimal places
  //   }
    
  //   Serial.println(); // Add a blank line to separate datasets
  //   delay(1000); // Wait a second before next sample
  // }
}