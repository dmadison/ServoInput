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
#include "ServoInput_Platforms.h"

#ifdef PCINT_VERSION
#define SERVOINPUT_USING_PCINTLIB
#endif


class ServoInputSignal {
public:
	ServoInputSignal();
	~ServoInputSignal();

	virtual boolean available() const = 0;

	uint16_t getPulse();
	virtual unsigned long getPulseRaw() const = 0;

	float getAngle();
	float getPercent();

	boolean getBoolean();

	long map(long outMin, long outMax);

	long mapDeadzone(long outMin, long outMax, float zonePercent);
	long mapDeadzonePulse(long outMin, long outMax, uint16_t zoneUs);

	uint16_t getRange() const;
	uint16_t getRangeMin() const;
	uint16_t getRangeMax() const;
	uint16_t getRangeCenter() const;

	void setRange(uint16_t range);
	void setRange(uint16_t min, uint16_t max);
	void setRangeMin(uint16_t min);
	void setRangeMax(uint16_t max);

	void resetRange();

	virtual uint8_t getPin() const = 0;

	static ServoInputSignal* getHead();
	ServoInputSignal* getNext() const;

protected:
	static const uint16_t PulseCenter = 1500;  // microseconds (us)
	static const uint16_t PulseValidRange = 2000;   // us ( 500 - 2500)
	static const uint16_t PulseDefaultRange = 1000;  // us (1000 - 2000)

	static boolean pulseValidator(unsigned long pulse);

	long remap(long pulse, long outMin, long outMax) const;

	uint16_t pulseMin, pulseMax;  // user-set range values

private:
	static ServoInputSignal* head;
	ServoInputSignal* next = nullptr;

	uint16_t lastPulse = 0;  // the last valid pulse
};


template<uint8_t Pin>
class ServoInputPin : public ServoInputSignal {
public:
	ServoInputPin() {
		ServoInputPin<Pin>::PinMask = PIN_TO_BITMASK(Pin);
		ServoInputPin<Pin>::Port = PIN_TO_BASEREG(Pin);
		pinMode(Pin, INPUT_PULLUP);

		attachInterrupt();
	}

	ServoInputPin(uint16_t pMin, uint16_t pMax) : ServoInputPin() {
		ServoInputSignal::setRange(pMin, pMax);
	}

	void attachInterrupt() {
		#if !defined(SERVOINPUT_NO_INTERRUPTS)
			#if !defined(SERVOINPUT_SUPPRESS_WARNINGS) && !defined(SERVOINPUT_USING_PCINTLIB)
				static_assert(digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT, "This pin does not support external interrupts!");
			#endif

			if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // if pin supports external interrupts
				::attachInterrupt(digitalPinToInterrupt(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
			}
			#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
			else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
				attachPCINT(digitalPinToPCINT(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
			}
			#endif
		#endif
	}

	void detachInterrupt() {
		#if !defined(SERVOINPUT_NO_INTERRUPTS)
			if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // detach external interrupt
				::detachInterrupt(digitalPinToInterrupt(Pin));
			}
			#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
			else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
				detachPCINT(digitalPinToPCINT(Pin));
			}
			#endif
		#endif
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

	boolean read() {
		unsigned long pulse = pulseIn(Pin, HIGH, 25000);  // 20 ms per + 5 ms of grace

		boolean validPulse = pulseValidator(pulse);
		if (validPulse == true) {
			pulseDuration = pulse;  // pulse is valid, store result
		}
		return validPulse;
	}

	unsigned long getPulseRaw() const {
		const unsigned long pulse = getPulseInternal();
		ServoInputPin<Pin>::changed = false;  // value has been read, is not longer 'new'
		return pulse;
	}

	uint8_t getPin() const {
		return Pin;
	}

	static void SERVOINPUT_ISR_FLAG isr() {
		static unsigned long start = 0;

		const boolean state = DIRECT_PIN_READ(Port, PinMask);

		if (state == HIGH) {  // rising edge
			start = micros();
		}
		else {  // falling edge
			pulseDuration = micros() - start;
			changed = true;
		}
	}

protected:
	static IO_REG_TYPE PinMask;
	static volatile IO_REG_TYPE* Port;

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

template<uint8_t Pin> IO_REG_TYPE ServoInputPin<Pin>::PinMask;
template<uint8_t Pin> volatile IO_REG_TYPE* ServoInputPin<Pin>::Port;

template<uint8_t Pin> volatile boolean ServoInputPin<Pin>::changed = false;
template<uint8_t Pin> volatile unsigned long ServoInputPin<Pin>::pulseDuration = 0;


class ServoInputManager {
public:
	static boolean available();
	static boolean allAvailable();
	static boolean anyAvailable();

	static uint8_t getNumSignals();
};

extern ServoInputManager ServoInput;

// Clean up platform-specific register definitions
#undef IO_REG_TYPE
#undef PIN_TO_BASEREG
#undef PIN_TO_BITMASK
#undef DIRECT_PIN_READ
#undef SERVOINPUT_ISR_FLAG

#endif
