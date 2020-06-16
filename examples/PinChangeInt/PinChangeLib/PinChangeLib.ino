/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *  @license    LGPLv3 - Copyright (c) 2020 David Madison
 *
 *  This example is part of the Arduino Servo Input Library.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Example:      PinChangeLib
 *  Description:  Uses a pin change interrupt (PCINT) in place of an external
 *                interrupt request. Allows you to use the ServoInput library
 *                with pins that don't support external interrupts.
 *
 *                Reference: https://playground.arduino.cc/Main/PinChangeInterrupt/
 *
 *                ---
 *
 *                Uses NicoHood's PinChangeInterrupt library to simplify the pin
 *                change interrupt setup when using multiple pin change interrupts
 *                on the same port.
 *
 *                Download: https://github.com/NicoHood/PinChangeInterrupt
 *
 *  Wiring:       Servo signal to pin 10, servo signal to pin 11
 *  Board:        Arduino Uno, Nano, or Mini (328P based)
 *                Arduino Leonardo, Micro, or Pro Micro (32U4 based)
 *                Arduino Mega
 */

#include <PinChangeInterrupt.h>  // must be included first!
#include <ServoInput.h>

// Neither of these pins support external interrupts on the Uno
// But they *will* work if you set up pin change interrupts!
const int pin1 = 10;  // PB2 / PCINT2 on Uno
const int pin2 = 11;  // PB3 / PCINT3 on Uno

ServoInputPin<pin1> servo1;
ServoInputPin<pin2> servo2;

void setup() {
	Serial.begin(115200);

	// wait for all servo signals to be read for the first time
	while (!ServoInput.available()) {
		Serial.println("Waiting for servo signals...");
		delay(500);
	}
}

void loop() {
	Serial.print("Servo Pin Change");

	// Print servo1 pin
	Serial.print(" (");
	Serial.print(servo1.getPin());
	Serial.print(") - ");

	// Print servo1 pulse, in microseconds (us)
	int pulse1 = servo1.getPulse();
	Serial.print(pulse1);
	Serial.print(" us");

	// Print servo2 pin
	Serial.print("| (");
	Serial.print(servo2.getPin());
	Serial.print(") - ");

	// Print servo2 pulse, in microseconds (us)
	int pulse2 = servo2.getPulse();
	Serial.print(pulse2);
	Serial.print(" us");

	Serial.println();
}
