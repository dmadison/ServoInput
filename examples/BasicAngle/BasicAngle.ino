/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *
 *  Example:      BasicAngle
 *  Description:  Reads the PWM signal from a servo motor, converts it to
 *                an angle, and prints that angle over Serial.
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

	servo.begin();
}

void loop() {
	float angle = servo.getAngle();  // get angle of servo (0-180)
	Serial.println(angle);
}
