Ultrasonic Hearing
==================

Enables you to extend your auditory sense into the ultrasonic range.
It can both be used with headphones or be connected directly to a cochlear implant.

This repository contains the code accompanying the master thesis `Ultrasonic Cyborg Hearing` by Maximilian Wagenbach pubished at the TU Berlin.
The most recent version of the code can be found at https://github.com/Foaly/UltrasonicHearing/.


Hardware
--------

This project uses the Teensy 4.1 and the corresponding Teensy 4.x audio shield.
However it can also be run on the Teensy 3.6 and its corresponding audio shield version.
The Knowles SPU0410LR5H-QB MEMS microphone is used as the ultrasound receiver.


License
-------

All code is licensed under the GNU General Public License (GPL) version 3.0.
For more information refer the file `LICENSE.txt`.


Execution
---------

To upload the software to the Teensy it has to be connected to the computer through a standard USB cable.
Afterwards the following commands have to be executed:

For Teensy 4.1 run: `pio run --target upload -e teensy41`

For Teensy 3.6 run: `pio run --target upload -e teensy36`


Tests
-----

Unit tests of the pitch shifter are provided.
They need to be run on the Teensy itself.
With the Teensy connected run the following command in the PlatformIO terminal:

For Teensy 4.1 run: `pio test -e teensy41`

For Teensy 3.6 run: `pio test -e teensy36`
