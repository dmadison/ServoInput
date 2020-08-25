# Servo Input Library
[![Build Status](https://travis-ci.com/dmadison/ServoInput.svg?branch=master)](https://travis-ci.com/dmadison/ServoInput)

This is an Arduino library that allows you to read the position of servo motors via their signal wire without delay. You can use this library to read RC receiver channels, find the motor positions of robotics, or debug other servo motor projects.

## Getting Started

```cpp
#include <ServoInput.h>

ServoInputPin<2> servo;

void setup() {}

void loop() {
	float angle = servo.getAngle();  // get angle of servo (0 - 180)
}
```

## Connecting Servos

Servo motors are driven by three pins: signal, power, and ground. To use this library, each 'signal' pin needs to be connected to an interrupt-capable I/O pin and the servo's ground wire must be tied to the Arduino's GND pin.

### Signals and Interrupts

The Servo Input library uses external interrupts to keep track of servo positions without delaying the rest of your sketch. In order for the library to work, you must connect the servo signal wires to [**interrupt-capable pins**](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/).

| BOARD                             | DIGITAL PINS USABLE FOR INTERRUPTS |
|-----------------------------------|------------------------------------|
| Uno, Nano, Mini, other 328-based  | 2, 3                               |
| Uno WiFi Rev.2                    | all digital pins                   |
| Mega, Mega2560, MegaADK           | 2, 3, 18, 19, 20, 21               |
| Micro, Leonardo, other 32u4-based | 0, 1, 2, 3, 7                      |
| Zero                              | all digital pins, except 4         |
| MKR Family boards                 | 0, 1, 4, 5, 6, 7, 8, 9, A1, A2     |
| Due                               | all digital pins                   |
| 101                               | 2, 5, 7, 8, 10, 11, 12, 13         |

*<sup>[Original table from arduino.cc/reference, modified to show only CHANGE interrupts](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)</sup>*

Some third party boards such as the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) and the [ESP8266](https://en.wikipedia.org/wiki/ESP8266) support external interrupts on all pins. Be sure to check the documentation for your board before connecting your servos.

Each servo's signal wire needs to be connected to a separate pin. If you do not have enough interrupt-capable pins on your board, you can also use [pin change interrupts](https://playground.arduino.cc/Main/PinChangeInterrupt/). ServoInput integrates with [NicoHood's](https://github.com/NicoHood) [PinChangeInterrupt](https://github.com/NicoHood/PinChangeInterrupt) library for AVRs - just import Nico's library before ServoInput and any necessary pin change interrupts will be added automatically. Be aware that due to the extra overhead pin change interrupts are not as accurate or reliable as external interrupts. Keep that in mind when choosing which pins to connect to which servo.

### Power Levels

Often servo motors are driven at higher voltages than your Arduino can handle. In this case, you will need either a level shifter or a voltage divider for the signal wire. Use a multimeter to check the servo's voltage level before making your connections.

***Warning:** if the servo's voltage is above the acceptable voltage for the Arduino, connecting the signal wire directly risks permanently damaging the board. Be smart and check before you connect anything.*

### Common Ground

If your servo is not powered by the Arduino running ServoInput, you must connect the ground wire from the servo's power supply to the ground pin of the Arduino so that they share a common reference.

If you are measuring multiple servos that use the same power supply, you do *not* need to connect the ground wire for each servo. Just the one is fine.

## License
This library is licensed under the terms of the [GNU Lesser General Public License (LGPL)](https://www.gnu.org/licenses/lgpl.html), either version 3 of the License, or (at your option) any later version.
