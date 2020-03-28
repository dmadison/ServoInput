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
 *  Example:      RC_Receiver
 *  Description:  Reads the steering and throttle servo signals from an RC
 *                receiver and prints them over Serial: steering as an angle
 *                from -90 to 90, and throttle as an integer percentage for
 *                both forwards and reverse.
 *
 *                This example requires 4 interrupt pins to work, and will
 *                *not* work without modification on the Arduino Uno and Nano.
 */

#include <ServoInput.h>

/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// Steering (CH1)
const int SteeringSignalPin = 0;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle (CH2)
const int ThrottleSignalPin = 1;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

// Button, on/off (CH3)
const int ButtonPin = 2;  // MUST be interrupt-capable!
ServoInputPin<ButtonPin> button;

// Slide Switch, 3-position (CH4)
const int SwitchPin = 3;  // MUST be interrupt-capable!
ServoInputPin<SwitchPin> slider;

void setup() {
	Serial.begin(115200);

	ServoInput.begin();

	while (!steering.available() && !throttle.available()) {
		Serial.println("Waiting for servo signals...");
		delay(500);
	}
}

void loop() {
	Serial.print("RC - ");

	// Steering: print as angle from -90 to 90
	float steeringAngle = steering.getAngle() - 90.0;  // returns 0 - 180, subtracting 90 to center at "0"
	Serial.print("Steering: ");
	Serial.print(steeringAngle);
	Serial.print("deg");

	Serial.print(" | ");  // separator

	// Throttle: print as percentage from -100 to 100
	int throttlePercent = throttle.map(-100, 100);  // remap to a percentage both forward and reverse
	Serial.print("Throttle: ");
	Serial.print(throttlePercent);
	Serial.print("% ");

	if (throttlePercent >= 0) {
		Serial.print("(Forward)");
	}
	else {
		Serial.print("(Reverse)");
	}

	Serial.print(" | ");  // separator

	// Button: print "X" if pressed
	boolean btn = button.getBoolean();  // reads the servo position as "on" if it's above the range's midpoint

	Serial.print("Button: ");

	if (btn == HIGH) {
		Serial.print("X");  // pressed!
	}
	else {
		Serial.print("_");  // not pressed
	}

	Serial.print(" | ");  // separator

	// Slide Switch: 3 position, show positions as 1-3
	uint8_t position = slider.map(1, 3);

	Serial.print("Slider: ");
	Serial.print(position);

	Serial.println();
}
