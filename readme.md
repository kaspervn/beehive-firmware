# Beehive experiment firmware #

Firmware for the SAMD21E18A-F microcontroller using Arduino and copied parts of the ASF from Microchip (in `lib/asf`).

## Available functions ##
Most of the Arduino functions are available. For reference to all the Arduino functions see [https://www.arduino.cc/reference/en](https://www.arduino.cc/reference/en).
Besides this we have functions to control the hardware on the Beehive board. See `src/hardware.h`

## How to compile and flash ##
Building the firmware files and uploading it to the microcontroller is easy thanks to [PlatformIO](https://platformio.org/).
Just install the PlatformIO plugin for [Visual Studio Code](https://code.visualstudio.com/) and open this folder as a project.
On linux you can use the commandline: `pio run -e experiment --target=upload` should be enough.

## Hardware testing ##

A test application is availabel that test the following hardware:
 - The coils are turned on for a few millisecond at full power. The sense resistor is read to determin the current.
 - The hall effect sensors are read out. It is checked wether a proper PWM in signal is detected

To flash this application on the hardware run `pio run -e hardware_test --target=upload`.
If you monitor the serial port you should see it sequantially tests the coils and the hall sensors.