# What's inside?
Firmware for the 2024 MIT Green Team's 2.009 final project. The software entails all power management, data transmission, signal processing, and hardware interface code. The main code can be found in the `ultrasonic_hearing` folder, forked from https://github.com/Foaly/UltrasonicHearing and adapted to run on the Teensy 4.0 Microcontroller.
# How to Use
In order to edit this code and run it on the Teensy with minimal inconvenience, it's recommended that you use the **Arduino IDE**. (*Aside: This is different from the old kinda ugly arduino editor like the ones on the 2.678 computers. Please install now if you don't have it already. It looks pretty like vscode*). 

To run the code on teensy and have all of your file interactions work properly, make sure that all of the `.ino`, `.cpp`, and `.h` or `.hpp` files inside of **a single arduino sketch**. 

Once that's taken care of, make sure you have **teensyduino** installed per this tutorial: https://www.pjrc.com/teensy/td_download.html

Select the **Teensy 4.0** as your board, the port it's plugged into, and you can get to writing code!

::::::::

*P.S. if you're on mac:*

You may need to install xcode or the xcode developer tools so you can compile your c++ code. What makes c++ different from python, for example, is that all of your code is generated *before* it runs, which is why it's so fast once you put it on a device. windows computers inherently can compile it, but mac needs something additional.

::::::::

Besides that, you should be able to get started with trying out your own code, stuff you find online, or the code inside of File -> Examples -> Examples for Teensy 4.0
# Generating Sound Files Using wav2sketch
If you would like to add a sound file corresponding to messages that the BC transducer should play, you can generate them using `wav2sketch`. It takes a few steps in order to go from a .wav file to something we can play on the teensy, but it's not too laborious.

**If you're on mac**:

1. Go to this link https://forum.pjrc.com/index.php?threads/instructions-or-tutorials-for-using-wav2sketch.42401/ and scroll until you find the `wav2sketch_mac.zip` OR go into the green team drive and look for `wav2sketch_mac` under the Learning Documents folder.

2. Once you have it downloaded, go into your terminal and navigate to the folder you want to run the `wav2sketch` file converter inside of. 

3. Move your .wav files into the same folder that `wav2sketch` is in. 
    
    **NOTE:** make sure the .wav files are recorded at/exported with a sample rate of 44.1 kHz. If you get an error about the sampling rate, you can drop your `.wav` file into **Audacity** (https://www.audacityteam.org) and export it with the proper sampling rate
3. Go into your terminal and type `chmod 755 wav2sketch`. This lets you execute the file from terminal. 
4. To run the file, type `./wav2sketch` and press enter. You should now see files of the format `AudioSample-SampleName` ending in `.cpp` and `.h` respectively. 
5. Go into the `.h` file and add the contents of your `.cpp` file. Only add the super long `const unsigned int` variable

```
#ifndef SAMPLENAME_H
#define SAMPLENAME_H // these should be in all caps

// ----- PASTE YOUR const unsigned int FROM THE .CPP FILE HERE ----- //

#endif 
```
6. Now, you can play your sound file through the teensy following the example of how we use `playMem` inside of `UltrasonicHearing.ino`.

