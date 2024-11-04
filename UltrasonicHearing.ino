//////////////////////////////////////////////////////////////////////
//
// Ultrasonic Hearing - Enabling you to extend your senses into
// the ultrasonic range using a Teensy 4.1
// Copyright (C) 2021    Maximilian Wagenbach
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.    If not, see <https://www.gnu.org/licenses/>.
//
//////////////////////////////////////////////////////////////////////


// Import default libraries
#include <Arduino.h>
#include <elapsedMillis.h>
#include <cmath>

//Import audio-related libraries
#include <Audio.h>
#include <Wire.h>

#include "PitchShift.hpp"
#include "DebugBlocks.hpp"
#include "WavFileWriter.hpp"
#include "utils.hpp"
#include "EncodeDecode.hpp"

// Import audio samples
#include "AudioSampleLow_o2.h"

// Import other device libraries
#include "Adafruit_LC709203F.h" //Battery monitor




/************ MESSAGE PACKETS */
union UnderwaterMessage {
    struct {
        uint8_t msg; // 8 bits for message
        uint8_t id;  // 8 bits for id
    };
    uint16_t data; // 16 bits total (concatenation of msg and id)
};

/************ MESSAGE QUEUEING */
//LIFO queue - last in first out
#define MESSAGE_QUEUE_LEN 10
UnderwaterMessage transmitMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t transmitMessagePointer = 0;

UnderwaterMessage receiveMessageQueue[MESSAGE_QUEUE_LEN];
uint8_t receiveMessagePointer = 0;

/********* */


/******* ADDITIONAL DEVICE SETUP */



const int micInput = AUDIO_INPUT_MIC;
//const int micInput = AUDIO_INPUT_LINEIN;

const uint32_t sampleRate = 44100;
//const uint32_t sampleRate = 96000;
// const uint32_t sampleRate = 192000;
//const uint32_t sampleRate = 234000;

const int16_t semitones = 12 * -4;    // shift in semitones
const float32_t pitchShiftFactor = std::pow(2., semitones / 12.);

elapsedMillis performanceStatsClock;


AudioControlSGTL5000      audioShield;
AudioInputI2S            audioInput;
PitchShift<2048>          fft(sampleRate, pitchShiftFactor);
Counter                    counter;
Printer                    printer;
AudioOutputI2S           audioOutput;
AudioOutputAnalogStereo    dacs1;            //xy=372,173
AudioSynthWaveformSine   sine;
AudioSynthNoiseWhite      noise;
AudioSynthWaveform       waveform1;        //xy=171,84
AudioSynthWaveform       waveform2;        //xy=178,148
AudioRecordQueue          queue;
WavFileWriter            wavWriter(queue);
AudioPlayMemory playMem; 

AudioAnalyzeRMS rms_L;
AudioAnalyzeRMS    rms_R;

int current_waveform=0;
//AudioConnection patchCord1(playMem, 0, audioOutput, 0); // thru hydro - 1, thru bone conduction - 0 for audioOut
// AudioConnection patchCord1(waveform1, 0, audioOutput, 1);
AudioConnection patchCord1(audioInput, 0, printer, 0); //for output: hydro - 0, bone conduction - 1    
// AudioConnection patchCord1(audioInput, 0, rms_L, 0); //for output: hydro - 0, bone conduction - 1    
// AudioConnection patchCord(audioInput, 0, rms_R, 1); //for output: hydro - 0, bone conduction - 1    
int dOUT;
int16_t* destination; 
// pinMode(dOUT, OUTPUT);



// setup for testing a whole octave of sine waves
float32_t octaveF10[13] = {22350.6, 23679.6, 25087.7, 26579.5, 28160.0, 29834.5, 31608.5, 33488.1, 35479.4, 37589.1, 39824.3, 42192.3};
// note names                  F 10,   F# 10,    G 10,   G# 10,    A 10,   A# 10,    B 10,    C 11,   C# 11,    D 11,   D# 11,    E 11
// AudioSynthWaveformSine sineBank[12];
// AudioMixer4 sineMixers[4];

// AudioConnection c1(sineBank[0], 0, sineMixers[0], 0);
// AudioConnection c2(sineBank[1], 0, sineMixers[0], 1);
// AudioConnection c3(sineBank[2], 0, sineMixers[0], 2);
// AudioConnection c4(sineBank[3], 0, sineMixers[0], 3);
// AudioConnection c5(sineBank[4], 0, sineMixers[1], 0);
// AudioConnection c6(sineBank[5], 0, sineMixers[1], 1);
// AudioConnection c7(sineBank[6], 0, sineMixers[1], 2);
// AudioConnection c8(sineBank[7], 0, sineMixers[1], 3);
// AudioConnection c9(sineBank[8], 0, sineMixers[2], 0);
// AudioConnection c10(sineBank[9], 0, sineMixers[2], 1);
// AudioConnection c11(sineBank[10], 0, sineMixers[2], 2);
// AudioConnection c12(sineBank[11], 0, sineMixers[2], 3);

// AudioConnection mix1(sineMixers[0], 0, sineMixers[3], 0);
// AudioConnection mix2(sineMixers[1], 0, sineMixers[3], 1);
// AudioConnection mix3(sineMixers[2], 0, sineMixers[3], 2);

// pass through
//AudioConnection    passThroughL(audioInput, 0, audioOutput, 0);
//AudioConnection    passThroughR(audioInput, 1, audioOutput, 1);

// debug printing
//AudioConnection    patchCord(counter, 0, fft, 0);
//AudioConnection    patchCord1(fft, 0, printer, 0);

//AudioConnection        sineToFFT(sine, 0, fft, 0);
//AudioConnection        noiseToFFT(noise, 0, fft, 0);
//AudioConnection        sineBankToFFT(sineMixers[3], 0, fft, 0);
AudioConnection        inToFFT(audioInput, 0, fft, 0);
AudioConnection        fftToOutL(fft, 0, audioOutput, 0);
AudioConnection        fftToOutR(fft, 0, audioOutput, 1);
//AudioConnection        micToWAV(audioInput, 0, queue, 0);
AudioConnection        fftToWav(fft, 0, queue, 0);
//AudioConnection        fftToPrinter(fft, 0, printer, 0);


void printPerformanceData();


void setup() {
    delay(500);
    Serial.begin(115200);
    AudioMemory(500);

    /******** INITIALIZATION */
    /*
    Steps:
    1) Initialize LED strip
        1b) Blink blue to show that we are alive!
    2) Initialize battery monitor
    3) Initialize audio shield
    4) Initialize I/O expander
    5) Play a quick tune on the bone conduction to show we are alive
    6) Blink green to show initialization good!
    If anything fails: display red on LED strip 
    */

   // Get LED strip up and running

   // Get battery monitor up and running
   Wire.setClock



    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(60);    //0-63
    audioShield.volume(1);    //0-1

    queue.begin(); 
    setI2SFreq(sampleRate);
    Serial.printf("Running at samplerate: %d\n", sampleRate);

    fft.setHighPassCutoff(20000.f);
    // pinMode(17, OUTPUT); //chirp the relays
    digitalWrite(17, LOW);

    // Confirgure both to use "myWaveform" for WAVEFORM_ARBITRARY
    // waveform1.arbitraryWaveform(myWaveform, 172.0);
    // waveform2.arbitraryWaveform(myWaveform, 172.0);

    // waveform1.frequency(440);
    // // waveform2.frequency(440);
    // waveform1.amplitude(1.0);
    // // waveform2.amplitude(1.0);

    // current_waveform = WAVEFORM_SQUARE;
    // waveform1.begin(current_waveform);

    sine.frequency(440.f * (AUDIO_SAMPLE_RATE_EXACT / sampleRate));
    sine.amplitude(0.2f);
    noise.amplitude(0.2f);

    Serial.println("Done initializing! Starting now!");
    // tone(14, 5000);
    playMem.play(AudioSampleLow_o2);
}


void loop() {
    // printer.update() ;
    // Serial.println(analogRead(8)); (
    //std::string s( buffer.data, buffer.size))
    //Serial.print(queue.readBuffer()); 
    //queue.printBuffer(); 
    
    memcpy(destination, queue.readBuffer(), 2);
    Serial.println(*destination); 
    // Serial.println("DESTINATION");
    // Serial.println(destination);


    // Serial.println("RMS LEFT");
    //Serial.println(rms_L.read()); 
    // Serial.println("RMS RIGHT");
    // Serial.println(rms_R.read()); 
    if(performanceStatsClock > 500) {
        printPerformanceData();
        performanceStatsClock = 0;
    }

    if (Serial.available() > 0) {
        // read the incoming byte
        byte incomingByte = Serial.read();

        if ( incomingByte == 'r' ) {
            if (!wavWriter.isWriting()) {
                std::size_t bufferSize = 6 + (sizeof(uint32_t) * 8) + 4 + 1;
                char filenameBuffer[bufferSize];
                snprintf(filenameBuffer, bufferSize, "ultra_%lu.wav", millis());

                Serial.print("Recording into \"");
                Serial.print(filenameBuffer);
                Serial.println("\" started!");
                wavWriter.open(filenameBuffer, sampleRate, 1);
            }
            else {
                Serial.println("Recording stopped!");
                wavWriter.close();
            }
        }
        if ( incomingByte == 's' ) {
            Serial.println("TRANSMITTING SINE WAVE");
            // current_waveform = WAVEFORM_SQUARE;
            playMem.play(AudioSampleLow_o2); //play verbal message
            // AudioNoInterrupts();
            // // // waveform1.begin(current_waveform); play regular wave
            
            // AudioInterrupts();
        }
    }

    if (wavWriter.isWriting())
        wavWriter.update();
}


void printPerformanceData() {
    Serial.print("CPU: ");
    Serial.print("fft=");
    Serial.print(fft.processorUsage());
    Serial.print(",");
    Serial.print(fft.processorUsageMax());
    Serial.print("    ");
    Serial.print("all=");
    Serial.print(AudioProcessorUsage());
    Serial.print(",");
    Serial.print(AudioProcessorUsageMax()); // estimate of the maximum percentage of CPU time any audio update has ever used (0 to 100)
    Serial.print("    ");
    Serial.print("Audio Memory: ");
    Serial.print(AudioMemoryUsage()); // number of audio blocks currently in use
    Serial.print(",");
    Serial.print(AudioMemoryUsageMax()); // maximum number of audio blocks ever in use
    Serial.println();
}