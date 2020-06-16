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
 *  Example:      Loopback
 *  Description:  Use the Servo library to set a servo position, then read
 *                that servo position using the Servo Input library.
 *
 *  Wiring:       Jumper between pins 9 (output) and 2 (input).
 */

#include <Servo.h>
#include <ServoInput.h>

/* The signal pin for ServoInput MUST be an interrupt-capable pin!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */
const int InputPin = 2;
const int OutputPin = 9;

ServoInputPin<InputPin> inputServo;
Servo outputServo;

int startPulse = 1000;  // microseconds (us)
int endPulse = 2000;    // microseconds (us)

int waitTime = 10;  // milliseconds (ms)

void setup() {
	Serial.begin(115200);

	outputServo.attach(OutputPin);  // attaches the servo on pin 9 to the servo object

	while (inputServo.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	// Going up!
	for (int pulseOut = startPulse; pulseOut < endPulse; pulseOut++) {
		outputServo.writeMicroseconds(pulseOut);
		printServoInfo(pulseOut);
		delay(waitTime);
	}

	// Going down!
	for (int pulseOut = endPulse; pulseOut > startPulse; pulseOut--) {
		outputServo.writeMicroseconds(pulseOut);
		printServoInfo(pulseOut);
		delay(waitTime);
	}
}

void printServoInfo(int pulseOut) {
	Serial.print("Servo Loopback | ");

	// Print the pulse duration, as written to the Servo
	Serial.print("W: ");
	Serial.print(pulseOut);
	Serial.print("  ");

	// Print the pulse duration, as read by ServoInput
	long pulseIn = inputServo.getPulseRaw();  // get unfiltered pulse duration
	Serial.print("R: ");
	Serial.print(pulseIn);
	Serial.print("  ");

	// Print the difference between the two!
	Serial.print("Diff: ");
	Serial.print(pulseIn - pulseOut);
	Serial.print(" us");
	Serial.println();
}
