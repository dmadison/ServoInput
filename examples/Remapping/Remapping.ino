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
 *  Example:      Remapping
 *  Description:  Reads a servo signal and then remaps it to angle, percent,
 *                and two arbitrary integer values.
 *
 *  Wiring:       Servo signal to pin 2
 */

#include <ServoInput.h>

/* The signal pin for ServoInput MUST be an interrupt-capable pin!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */
ServoInputPin<2> servo;

void setup() {
	Serial.begin(115200);

	while (servo.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	Serial.print("Servo Map | ");

	// Use the built-in angle map (float, 0 - 180)
	float angle = servo.getAngle();

	Serial.print("A: ");
	Serial.print(angle);
	Serial.print("deg");

	Serial.print("  ");

	// Use the built-in percentage map (float, 0 - 1)
	float percent = servo.getPercent();

	Serial.print("P: ");
	Serial.print(percent);

	Serial.print(" | ");

	// Use the built-in boolean map (signal duration HIGH or LOW)
	boolean state = servo.getBoolean();

	Serial.print("Bool: ");
	if (state == true) {
		Serial.print("HIGH");
	}
	else if (state == false) {
		Serial.print("LOW");
	}

	Serial.print(" | ");

	// Map to a single byte, unsigned
	byte singleByte = servo.map(0, 255);

	Serial.print("Byte: ");
	Serial.print(singleByte);

	Serial.print("  ");

	// Map to a 16-bit signed integer
	int signedInt = servo.map(-32768, 32767);

	Serial.print("Int: ");
	Serial.print(signedInt);

	Serial.print(" ");

	// Map with a deadzone in the center
	float deadzoneSize = 0.25;  // 25% around the center
	int deadzoneMap = servo.mapDeadzone(-1000, 1000, deadzoneSize);

	Serial.print("Deadzone: ");
	Serial.print(deadzoneMap);

	Serial.println();
}
