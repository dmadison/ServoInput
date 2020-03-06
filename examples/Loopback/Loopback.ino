/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *
 *  Example:      Loopback
 *  Description:  Use the Servo library to set a servo position, then read
 *                that servo position using the Servo Input library.
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
ServoInputPin<2> servoInput;
Servo servoOutput;

int startPulse = 1000;  // microseconds (us)
int endPulse = 2000;    // microseconds (us)

int waitTime = 10;  // milliseconds (ms)

void setup() {
	Serial.begin(115200);

	servoInput.begin();     // sets the pin states and interrupt for the servo input
	servoOutput.attach(9);  // attaches the servo on pin 9 to the servo object

	while (servoInput.available() == false) {
		Serial.println("Waiting for servo signal...");
		delay(500);
	}
}

void loop() {
	// Going up!
	for (int pulseOut = startPulse; pulseOut < endPulse; pulseOut++) {
		servoOutput.writeMicroseconds(pulseOut);
		printServoInfo(pulseOut);
		delay(waitTime);
	}

	// Going down!
	for (int pulseOut = endPulse; pulseOut > startPulse; pulseOut--) {
		servoOutput.writeMicroseconds(pulseOut);
		printServoInfo(pulseOut);
		delay(waitTime);
	}
}

void printServoInfo(int pulseOut) {
	Serial.print("Servo Loopback | ");

	// Print the pulse duration, as written to the Servo object
	Serial.print("W: ");
	Serial.print(pulseOut);
	Serial.print("  ");

	// Print the pulse duration, as read by the ServoInput object
	long pulseIn = servoInput.getPulseRaw();  // get unfiltered pulse duration
	Serial.print("R: ");
	Serial.print(pulseIn);
	Serial.print("  ");

	// Print the difference between the two!
	Serial.print("Diff: ");
	Serial.print(pulseIn - pulseOut);
	Serial.print(" us");
	Serial.println();
}
