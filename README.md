# Servo Input Library

This is an Arduino library that allows you to read the position of servo motors via their signal wire without delay. You can use this library to read RC receiver channels, find the motor positions of robotics, or debug other servo motor projects.

## Getting Started

```cpp
#include <ServoInput.h>

ServoInputPin<2> servo;  // must be an interrupt-capable pin!

void setup() {
	ServoInput.begin();
}

void loop() {
	float angle = servo.getAngle();  // get angle of servo (0 - 180)
}
```

## License
This library is licensed under the terms of the [GNU Lesser General Public License (LGPL)](https://www.gnu.org/licenses/lgpl.html), either version 3 of the License, or (at your option) any later version.
