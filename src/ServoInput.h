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

#ifdef PCINT_VERSION
#define SERVOINPUT_USING_PCINTLIB
#endif

class ServoInputManager {
public:
	static void begin();
	static void end();
};

extern ServoInputManager ServoInput;


class ServoInputSignal {
public:
	friend class ServoInputManager;

	ServoInputSignal();
	~ServoInputSignal();

	virtual void begin() = 0;
	virtual void end() = 0;

	virtual boolean available() const = 0;

	uint16_t getPulse() const;
	virtual unsigned long getPulseRaw() const = 0;

	float getAngle() const;
	float getPercent() const;

	boolean getBoolean() const;

	long map(long outMin, long outMax) const;

	uint16_t getRange() const;
	uint16_t getRangeMin() const;
	uint16_t getRangeMax() const;
	uint16_t getRangeCenter() const;

	void setRange(uint16_t range);
	void setRange(uint16_t min, uint16_t max);
	void setRangeMin(uint16_t min);
	void setRangeMax(uint16_t max);

	void resetRange();

protected:
	static const uint16_t PulseCenter = 1500;  // microseconds (us)
	static const uint16_t PulseValidRange = 1200;   // us, +/- ( 300 - 2700)
	static const uint16_t PulseDefaultRange = 500;  // us, +/- (1000 - 2000)

	static boolean pulseValidator(unsigned long pulse);

	long remap(long pulse, long outMin, long outMax) const;

	uint16_t pulseMin, pulseMax;  // user-set range values

	static ServoInputSignal* head;
	static ServoInputSignal* tail;
	ServoInputSignal* next;
};


template<uint8_t Pin>
class ServoInputPin : public ServoInputSignal {
public:
	ServoInputPin() {}
	ServoInputPin(uint16_t pMin, uint16_t pMax) {
		ServoInputSignal::setRange(pMin, pMax);
	}

	void begin() {
		#if !defined(SERVOINPUT_SUPPRESS_WARNINGS) && !defined(SERVOINPUT_USING_PCINTLIB)
			static_assert(digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT, "This is not an interrupt-capable pin!");
		#endif

		ServoInputPin<Pin>::PinMask = digitalPinToBitMask(Pin);
		ServoInputPin<Pin>::Port = portInputRegister(digitalPinToPort(Pin));
		pinMode(Pin, INPUT_PULLUP);

		if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // if pin supports external interrupts
			attachInterrupt(digitalPinToInterrupt(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
		}
		#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
		else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
			attachPCINT(digitalPinToPCINT(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
		}
		#endif
	}

	void end() {
		if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // detach external interrupt
			detachInterrupt(digitalPinToInterrupt(Pin));
		}
		#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
		else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
			detachPCINT(digitalPinToPCINT(Pin));
		}
		#endif
	}

	static uint8_t pin() {
		return Pin;
	}

	static void isr() {
		static unsigned long start = 0;

		const boolean state = (*Port & PinMask) != 0;

		if (state == HIGH) {  // rising edge
			start = micros();
		}
		else {  // falling edge
			pulseDuration = micros() - start;
			changed = true;
		}
	}

	boolean available() const {
		boolean change = ServoInputPin<Pin>::changed;  // store temp version of volatile flag

		if (change == true) {
			boolean pulseValid = ServoInputSignal::pulseValidator(getPulseInternal());

			if (pulseValid == false) {
				ServoInputPin<Pin>::changed = change = false;  // pulse is not valid, so we can reset (ignore) the 'changed' flag
			}
		}
		return change;
	}

	unsigned long getPulseRaw() const {
		const unsigned long pulse = getPulseInternal();
		ServoInputPin<Pin>::changed = false;  // value has been read, is not longer 'new'
		return pulse;
	}

protected:
	static uint8_t PinMask;
	static volatile uint8_t* Port;

	static volatile boolean changed;
	static volatile unsigned long pulseDuration;

	static unsigned long getPulseInternal() {
		// disable / enable interrupts here so the multi-byte variable is not
		// updated while it's being copied from volatile memory
		noInterrupts();
		const unsigned long pulse = ServoInputPin<Pin>::pulseDuration;
		interrupts();
		return pulse;
	}
};

template<uint8_t Pin> uint8_t ServoInputPin<Pin>::PinMask;
template<uint8_t Pin> volatile uint8_t* ServoInputPin<Pin>::Port;

template<uint8_t Pin> volatile boolean ServoInputPin<Pin>::changed = false;
template<uint8_t Pin> volatile unsigned long ServoInputPin<Pin>::pulseDuration = 0;

#endif
