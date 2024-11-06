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

//Import encoding/decoding libraries
#include <iostream>

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
#include "AudioSampleSos.h"
#include "AudioSampleGoing_up.h"
#include "AudioSampleGoing_down.h"
#include "AudioSampleLow_oxygen.h"
#include "AudioSampleCheck_in.h"
#include "AudioSampleCome_look.h"
#include "AudioSampleNo_msg.h"

// Import other device libraries
#include <Adafruit_LC709203F.h> // Battery monitor
#include <Adafruit_MCP23X17.h> // IO expander
#include <Adafruit_NeoPixel.h> // LEDs

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

/******* ADDITIONAL DEVICE SETUP */

// IO expander, buttons, LEDs set up
#define BUTTON_PIN1 0
#define BUTTON_PIN2 1
#define BUTTON_PIN3 2
#define BUTTON_PIN4 3
#define BUTTON_PIN5 4
#define BUTTON_PIN6 5
#define BUTTON_PIN7 6

// Pin connected to neopixels strip
#define LED_PIN  14
#define LED_COUNT 12

// Declare NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ800);

uint32_t magenta = strip.Color(255, 0, 255, 0);
uint32_t greenishwhite = strip.Color(0, 64, 0, 64); // r g b w
uint32_t bluishwhite = strip.Color(64, 0, 0, 64);

Adafruit_MCP23X17 mcp;

uint8_t row_pins[4] = {BUTTON_PIN2, BUTTON_PIN7, BUTTON_PIN6, BUTTON_PIN4};
uint8_t col_pins[3] = {BUTTON_PIN3, BUTTON_PIN1, BUTTON_PIN5};

char keypad_array[4][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 0, 11}};
static const char *message_array[4][3] = {{"SOS", "GOING UP", "GOING DOWN"}, {"LOW OXYGEN", "CHECK-IN", "COME LOOK"}, {"no msg", "no msg", "no msg"}, {"no msg", "no msg", "no msg"}};
// replace message array with audio files (NOTE: evan & OT - we need to go through and make these of type UnderwaterMessage or do some conversion)

const unsigned int *audio_messages_array[4][3] = {
  {AudioSampleSos, AudioSampleGoing_up, AudioSampleGoing_down},
  {AudioSampleLow_oxygen, AudioSampleCheck_in, AudioSampleCome_look},
  {AudioSampleNo_msg, AudioSampleNo_msg, AudioSampleNo_msg}
};

// amplifier set ups

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
AudioPlayMemory           playMem; 
AudioAnalyzeToneDetect    findTone;
AudioAnalyzeRMS           rms_L;
AudioAnalyzeRMS           rms_R;

int current_waveform=0;
AudioConnection bc_transducer(playMem, 0, audioOutput, 0); // thru hydro - 1, thru bone conduction - 0 for audioOut
// AudioConnection patchCord1(waveform1, 0, audioOutput, 1);
AudioConnection hydro_listener(audioInput, 0, findTone, 0); //for output: hydro - 0, bone conduction - 1    
AudioConnection hydro_listener1(audioInput, queue); 
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

enum OperatingMode { 
    RECEIVE,
    TRANSMIT,
    ERROR
};
enum OperatingMode mode;

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

    // Get LEDs up and running
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    strip.setBrightness(32);

    // LEDs: show that we are alive
    strip.fill(bluishwhite, 0, 8); // light up entire strip
    strip.show();
    // keep strip on only for 3 seconds, then continue with rest of installation
    // change the delay to while?
    delay(3000); 
    strip.clear();

    // Get IO expander up and running
    if (!mcp.begin_I2C()) {
        Serial.println("Error: I2C connection with IO expander.");
        initializationError(1);
    }

    // Get buttons (keypad right now) up and running    
    for (int r = 0; r < 4; r++) {
        mcp.pinMode(row_pins[r], OUTPUT);
        mcp.digitalWrite(row_pins[r], HIGH);
        }
    for (int c = 0; c < 3; c++) {
        mcp.pinMode(col_pins[c], INPUT_PULLUP);
        }
    // Serial.println("Begin loop to check what buttons are pressed.");


    // Get battery monitor up and running
    Wire.setClock(100000);
    // initializationError(2);
    

    // Get audio shield up and running
    audioShield.enable();
    audioShield.inputSelect(micInput);
    audioShield.micGain(60);    //0-63
    audioShield.volume(1);    //0-1
    // initializationError(3);

    queue.begin(); 
    setI2SFreq(sampleRate);
    Serial.printf("Running at samplerate: %d\n", sampleRate);

    fft.setHighPassCutoff(20000.f);
    pinMode(17, OUTPUT); //set relay pin as output


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
    // playMem.play(AudioSampleLow_o2);

    //Battery check setup
    if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
    }
    Serial.println(F("Found LC709203F"));
    Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

    lc.setThermistorB(3950);
    Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

    lc.setPackSize(LC709203F_APA_500MAH);

    lc.setAlarmVoltage(3.8);

    mode = RECEIVE; // set it to receiving mode
}

static long time = 0;
//init time for non-blocking check

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

    strip.clear(); // Set all pixel colors to 'off'

    for (int r = 0; r < 4; r++) {
        mcp.digitalWrite(row_pins[r], LOW);
        delay(5);
        for (int c = 0; c < 3; c++) {
            if (mcp.digitalRead(col_pins[c]) == LOW) {
                mode = TRANSMIT;
                Serial.println(keypad_array[r][c]); // returns button pressed
                Serial.println(message_array[r][c]); // returns message pressed

                transmitMessageQueue[transmitMessagePointer] = message_array[r][c];// add msg to queue 
                transmitMessagePointer++; //increment ptr by one 

                // confirmation of message pressed back to bone conduction
                playMem.play(audio_messages_array[r][c]); // plays message pressed

                strip.fill(magenta, 0, keypad_array[r][c]);
                strip.show();
            }
        }
        mcp.digitalWrite(row_pins[r], HIGH);
        delay(5);
        }
    switch(mode) {
        case RECEIVE:
            update_relays(mode);
            receive();
            break;
        case TRANSMIT:
            update_relays(mode);
            transmit();
            break;
    }
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

    if (millis() - time > 2000){
        printBatteryData();

    }
    //printing battery status every 2 seconds, displaying on neopixel
    
}

// TODO otti moment
void initializationError(int error) {
    // passed in int error represents number of LEDs in strip that we want to light up
    // for initialization, using greenishwhite (whereas showing life is bluishwhite)
    
    strip.fill(greenishwhite, 0, error); // error = number of tiles lit up
    strip.show();

    while(1); // how long to keep on for?
    strip.clear();
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


//colors
uint32_t green = strip.Color(0, 255, 0);
uint32_t red = strip.Color(255, 0, 0);
uint32_t orange = strip.Color(255, 150, 0);
uint32_t yellow = strip.Color(255, 255, 0);
void printBatteryData(){
    time = millis()
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
        strip.fill(orange, 0, 3)
    }
    else{
        strip.fill(red, 0, 1)
    }
    strip.show()
}

void update_relays(OperatingMode newMode) {
    if (newMode == RECEIVE); {
        digitalWrite(17, LOW); // make relays go to listen mode
    }
    if (newMode == TRANSMIT) {
        digitalWrite(17, HIGH); // make relays go into transmit mode
    }
}

void receive() {
    if (findTone(start_freq)) {
        Serial.println("FOUND START FREQUENCY");
    } // start recording upon receipt of start frequency
    int[32] inData; 
    for (i=0; i < 32; i++) {
        if (findTone(low_freq)) {
            inData[i] = 0;
        }
        else if (findTone(high_freq)) {
            inData[i] = 1;
        }
        if (findtone(end_freq)) {
            Serial.println("FOUND END OF MESSAGE"); 
            break;
        }
    }
    /* TODO: Also implement version that reads data from the buffer then decodes it */
    // queue.clear()
    // queue.readBuffer()
    
    // Now process the signal
    decoded_msg = decode_msg(inData);
    mapped_msg = message_map[decoded_msg];

    receiveMessageQueue[mapped_msg]; // add received message to the queue 
    receiveMessagePointer++;

    playMem(receiveMessageQueue[receiveMessagePointer]);
    receiveMessagePointer--; // after playing through BC transducer decrement position in queue by 1

    check_pointers(&receiveMessagePointer);


}

void play_data(int[32] inData); {
    // frequencies of the message in kHz;
    OUT_PIN = 14; 
    tone_duration = 500; // play each tone for 500ms 
    
    tone(OUT_PIN, start_freq, tone_duration); 

    for (int i=0, i < 32, i++) {
        if (inData[i] == 0) {
            tone(OUT_PIN, low_freq, tone_duration);
        }
        else if (indata[i] == 1) {
            tone(OUT_PIN, high_freq, tone_duration);
        }
    }

    tone(OUT_PIN, end_freq, tone_duration); //wrap it up with the end frequency
}
void transmit() {
    /* Plays a single message through the hydrophone, then switches back to receiving mode */
    int[32] outputData; // init 32 bit array of ints
    UnderwaterMessage msg = transmitMessageQueue[transmitMessagePointer]; // message is current item in the queue
    msg.id = 3; // arbitrary diver ID 
    encode(msg, &outputData);  // outputData now has message inside of it
    play_data(outputData); //play that message 

    mode = RECEIVE; // switch back to receiving mode 
    transmitMessagePointer--; // decrement pointer by 1 to get next newest message in queue 
    return; 
}

void check_pointers(uint8_t &pointer) {
    if (pointer > 0) { // wrap back around if overshoot queue len
        pointer = pointer % MESSAGE_QUEUE_LEN;
    }
    else if (pointer < 0) {
        pointer = 0; // floor pointer index at 0
    }
}
