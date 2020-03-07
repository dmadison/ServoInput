/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *  @license    LGPLv3 - Copyright (c) 2020 David Madison
 *
 *  This file is part of the Arduino Servo Input Library.
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
 */

#ifndef ServoInput_h
#define ServoInput_h

#include <Arduino.h>

class ServoInputSignal {
public:
	ServoInputSignal();
	ServoInputSignal(uint16_t pMin, uint16_t pMax);
	~ServoInputSignal();

	virtual void begin() = 0;
	virtual void end() = 0;

	boolean available() const;

	uint16_t getPulse() const;
	virtual unsigned long getPulseRaw() const = 0;

	float getAngle() const;
	float getPercent() const;

	long map(long outMin, long outMax) const;

	uint16_t getRange() const;
	uint16_t getRangeMin() const;
	uint16_t getRangeMax() const;
	uint16_t getRangeCenter() const;

	void setRange(uint16_t range);
	void setRange(uint16_t min, uint16_t max);
	void setRangeMin(uint16_t min);
	void setRangeMax(uint16_t max);

protected:
	static const uint16_t MinValidPulse = 300;   // us, 1500 - 1200
	static const uint16_t MaxValidPulse = 2700;  // us, 1500 + 1200
	static boolean pulseValidator(unsigned long pulse);

	long remap(long pulse, long outMin, long outMax) const;

	uint16_t pulseMin = 1000;  // 1500 us center - 500 us
	uint16_t pulseMax = 2000;  // 1500 us center + 500 us

	static ServoInputSignal* head;
	static ServoInputSignal* tail;
	ServoInputSignal* next;
};


template<uint8_t Pin>
class ServoInputPin : public ServoInputSignal {
public:
	ServoInputPin() {}
	ServoInputPin(uint16_t pMin, uint16_t pMax) : ServoInputSignal(pMin, pMax) {}

	void begin() {
		static_assert(digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT, "This is not an interrupt-capable pin!");
		ServoInputPin<Pin>::PinMask = digitalPinToBitMask(Pin);
		ServoInputPin<Pin>::Port = portInputRegister(digitalPinToPort(Pin));
		pinMode(Pin, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
	}

	void end() {
		detachInterrupt(digitalPinToInterrupt(Pin));
	}

	static uint8_t pin() {
		return Pin;
	}

	static void isr() {
		static unsigned long start = 0;

		const boolean state = (*Port & PinMask) != 0;

		if (state == HIGH) start = micros(); // rising edge
		else pulseDuration = micros() - start;  // falling edge
	}

	unsigned long getPulseRaw() const {
		noInterrupts();
		const unsigned long pulse = ServoInputPin<Pin>::pulseDuration;
		interrupts();
		return pulse;
	}

protected:
	static uint8_t PinMask;
	static volatile uint8_t* Port;

	static volatile unsigned long pulseDuration;
};

template<uint8_t Pin> uint8_t ServoInputPin<Pin>::PinMask;
template<uint8_t Pin> volatile uint8_t* ServoInputPin<Pin>::Port;
template<uint8_t Pin> volatile unsigned long ServoInputPin<Pin>::pulseDuration = 0;

#endif
