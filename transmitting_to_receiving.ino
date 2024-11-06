/* SPH0645 MEMS Microphone Test (Adafruit product #3421)
 *
 * Forum thread with connection details and other info:
 * https://forum.pjrc.com/threads/60599?p=238070&viewfull=1#post238070
 */
#include <Audio.h>
// const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;
// GUItool: begin automatically generated code


AudioControlSGTL5000 audioShield;
AudioInputI2S            i2s1;           //xy=180,111
// AudioFilterStateVariable filter1;        //xy=325,101
AudioFilterBiquad        biquad1;
AudioAmplifier           amp1;           //xy=470,93
AudioAnalyzeFFT1024      fft1024_1;      //xy=616,102
// AudioConnection          patchCord1(i2s1, 0, filter1, 0);
AudioConnection          patchCord1(i2s1, 0, amp1, 0);
// AudioConnection          patchCord2(filter1, 2, amp1, 0);
AudioConnection          patchCord2(amp1, 0, biquad1, 0);
AudioConnection          patchCord3(biquad1, 0, fft1024_1, 0);
// AudioAnalyzePeak        peak; 
// AudioConnection         patchCord4(amp1, peak); 
// GUItool: end automatically generated code
void setup() {
  
  pinMode(14, OUTPUT); 
  AudioMemory(50);
  // filter1.frequency(30); // filter out DC & extremely low frequencies
  biquad1.setBandpass(0,16000,15);//Filter to only freq between 950-1050
  // for (int i=1; i< 3; i++) {
  //   biquad1.setBandpass(i, 16000, 0.7071);
  // }
  amp1.gain(2);        // amplify sign to useful range
  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.micGain(80);
  audioShield.volume(1);
  Serial.println("Setup done");
}
int curTone = 5000;
unsigned long lastToneChange = 0;
void loop() {
  delay(100);

  if (millis() - lastToneChange > 5000) {
    lastToneChange = millis();

    noTone(14);
    tone(14, curTone); 

    if (curTone == 5000) {
      curTone = 15000;
    } else {
      curTone = 5000;
    }

    // curTone = (curTone == 5000) ? 15000 : 5000;
  }

  if (fft1024_1.available()) {
    // each time new FFT data is available
    Serial.print("FFT: ");
    double maxBinAmp = 0;
    double maxBinHz = 0;
    for (int i = 0; i < 1024; i++) {
      float n = fft1024_1.read(i);
      if (n > maxBinAmp) {
        maxBinAmp = n;
        maxBinHz = (double)47.0 * (double)i;
      }
    }

    if (maxBinAmp > 0.01) {
      Serial.print("Amp: ");
      Serial.print(maxBinAmp);
      Serial.print(" @ ");
      Serial.print(maxBinHz);
      Serial.println("Hz");
    } else {
      Serial.println("Amplitude too low");
    }
  }
  // if (peak.available()) { 
  //   Serial.println(peak.read()); 
  // }

}