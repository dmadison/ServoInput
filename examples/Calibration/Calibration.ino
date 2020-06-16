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
 *  Example:      Calibration
 *  Description:  Reads the PWM signal from a servo motor and automatically
 *                records the min and max values for calibration. The min, max,
 *                and current pulse width values are then sent over serial in
 *                an easy-to-read format.
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

	int center = servo.getRangeCenter();  // get center value of range
	servo.setRange(center, center);  // set min/max values to center

	while (servo.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	// Get servo signal pulse length, in microseconds (unfiltered)
	const uint16_t pulse = (uint16_t) servo.getPulseRaw();

	// Check + store range min/max
	if (pulse < servo.getRangeMin()) {
		servo.setRangeMin(pulse);
	}
	else if (pulse > servo.getRangeMax()) {
		servo.setRangeMax(pulse);
	}

	// Print results using sprintf for easy formatting
	char buffer[100];
	sprintf(buffer, "Servo PWM (us) | Min: %4u  Val: %4u  Max: %4u | Range: %4u", 
		servo.getRangeMin(), pulse, servo.getRangeMax(), servo.getRange());
	Serial.println(buffer);
}
