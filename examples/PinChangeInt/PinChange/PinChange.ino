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
 *  Example:      PinChange
 *  Description:  Uses a pin change interrupt (PCINT) in place of an external
 *                interrupt request. Allowing you to use the ServoInput library
 *                with pins that don't support external interrupts.
 *
 *                Fair warning: this gets far more complex if you use more than
 *                one pin change interrupt with the same group, because you will
 *                need to determine which pin triggered the interrupt before you
 *                call that pin's ISR.
 *
 *                Reference: https://playground.arduino.cc/Main/PinChangeInterrupt/
 *
 *  Wiring:       Servo signal to pin 9
 *  Board:        Arduino Uno, Nano, or Mini (328P based)
 */

#define SERVOINPUT_SUPPRESS_WARNINGS  // enables the use of pin change interrupts in the library
#include <ServoInput.h>

#if !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega328P__)
#error This sketch is written for the ATmega328P (Uno, Nano, Mini, etc.) Other microcontrollers may not work. Comment out this line to continue.
#endif

const int pin = 9;  // NOT an interrupt-capable pin! But will still work if you set a pin change interrupt
ServoInputPin<pin> servo;


void setInterrupt() {
	*digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin  (pin D9: PCMSK0 |= 1 << PCINT1)
	PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt (pin D9: PCIFR |= 1 << PCIF0)
	PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group (pin D9: PCICR |= 1 << PCIF0)
 }

ISR(PCINT0_vect) {  // pin change ISR handler for Arduino Uno pins D8 - D13
	servo.isr();
}


void setup() {
	Serial.begin(115200);

	setInterrupt();  // set pin change interrupt (see above)

	while (servo.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	Serial.print("Servo Pin Change (");
	Serial.print(pin);
	Serial.print(") - ");

	int pulse = servo.getPulse();  // get servo pulse duration, in microseconds (us)
	Serial.print(pulse);
	Serial.print(" us");

	Serial.println();
}
