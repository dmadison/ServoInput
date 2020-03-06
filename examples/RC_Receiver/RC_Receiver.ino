/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *
 *  Example:      RC_Receiver
 *  Description:  Reads the steering and throttle servo signals from an RC
 *                receiver and prints them over Serial: steering as an angle
 *                from -90 to 90, and throttle as an integer percentage for
 *                both forwards and reverse.
 */

#include <ServoInput.h>

/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// Steering Setup
const int SteeringSignalPin = 2;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1000;  // microseconds (us)
const int SteeringPulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

// Throttle Setup
const int ThrottleSignalPin = 3;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1000;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);
ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

void setup() {
	Serial.begin(115200);

	steering.begin();
	throttle.begin();

	while (!steering.available() && !throttle.available()) {
		Serial.println("Waiting for servo signals...");
		delay(500);
	}
}

void loop() {
	Serial.print("RC - ");

	float steeringAngle = steering.getAngle() - 90.0;  // returns 0 - 180, subtracting 90 to center at "0"
	Serial.print("Steering: ");
	Serial.print(steeringAngle);
	Serial.print("deg");

	Serial.print(" | ");  // separator

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

	Serial.println();
}
