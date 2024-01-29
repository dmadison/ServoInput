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
	virtual ~ServoInputSignal();

	virtual bool available() const = 0;

	uint16_t getPulse();
	virtual unsigned long getPulseRaw() const = 0;

	float getAngle();
	float getPercent();

	bool getBoolean();

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

	static bool pulseValidator(unsigned long pulse);

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
		#ifdef SERVOINPUT_PIN_SPECIALIZATION
			ServoInputPin<Pin>::PinMask = SERVOINPUT_PIN_TO_BITMASK(Pin);
			ServoInputPin<Pin>::PortRegister = SERVOINPUT_PIN_TO_BASEREG(Pin);
		#endif
		pinMode(Pin, INPUT_PULLUP);
		
		ServoInputPin<Pin>::refCount++;
	}

	ServoInputPin(uint16_t pMin, uint16_t pMax) : ServoInputPin() {
		ServoInputSignal::setRange(pMin, pMax);
	}

	~ServoInputPin() {
		ServoInputPin<Pin>::refCount--;
		if (ServoInputPin<Pin>::refCount == 0) {
			this->detach();  // no more class instances, detach interrupt
		}
	}

	void attach() {
		#if !defined(SERVOINPUT_NO_INTERRUPTS)

			// Compile-time check that the selected pin supports interrupts
			#if !defined(SERVOINPUT_DISABLE_PIN_CHECK) && !defined(SERVOINPUT_SUPPRESS_WARNINGS) && !defined(SERVOINPUT_USING_PCINTLIB)
				static_assert(digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT, "This pin does not support external interrupts!");
			#endif

			// quit early if the interrupt is already attached
			if (ServoInputPin<Pin>::interruptAttached == true) return;

			// Interrupt attachment, with pin checks
			#if !defined(SERVOINPUT_DISABLE_PIN_CHECK)

				// Interrupt attachment, platform support
				if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // if pin supports external interrupts
					attachInterrupt(digitalPinToInterrupt(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
				}

				// Interrupt attachment, PinChangeInterrupt
				#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
				else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
					attachPCINT(digitalPinToPCINT(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
				}
				#endif

			// Interrupt attachment, no pin checks
			// note that the PinChangeInterrupt library isn't supported here,
			// because we have no way of checking whether the pin is supported
			// in hardware vs in the library
			#else
				attachInterrupt(digitalPinToInterrupt(Pin), reinterpret_cast<void(*)()>(isr), CHANGE);
			#endif

			// set flag to avoid multiple attachment requests
			ServoInputPin<Pin>::interruptAttached = true;

		#endif
	}

	void detach() {
		#if !defined(SERVOINPUT_NO_INTERRUPTS)

			// quit early if the interrupt is not attached
			if (ServoInputPin<Pin>::interruptAttached == false) return;

			// Interrupt detachment, with pin checks
			#if !defined(SERVOINPUT_DISABLE_PIN_CHECK)

				// Interrupt detachment, platform support
				if (digitalPinToInterrupt(Pin) != NOT_AN_INTERRUPT) {  // detach external interrupt
					detachInterrupt(digitalPinToInterrupt(Pin));
				}

				// Interrupt detachment, PinChangeInterrupt
				#if defined(SERVOINPUT_USING_PCINTLIB)  // if using NicoHood's PinChangeInterrupt library
				else if (digitalPinToPCINT(Pin) != NOT_AN_INTERRUPT) {
					detachPCINT(digitalPinToPCINT(Pin));
				}
				#endif

			// Interrupt detachment, no pin checks
			#else
				detachInterrupt(digitalPinToInterrupt(Pin));
			#endif

			// set flag to show that we've detached successfully
			ServoInputPin<Pin>::interruptAttached = false;

		#endif
	}

	bool available() const {
		bool change = ServoInputPin<Pin>::changed;  // store temp version of volatile flag

		if (change == true) {
			bool pulseValid = ServoInputSignal::pulseValidator(getPulseInternal());

			if (pulseValid == false) {
				ServoInputPin<Pin>::changed = change = false;  // pulse is not valid, so we can reset (ignore) the 'changed' flag
			}
		}
		return change;
	}

	bool read() {
		unsigned long pulse = pulseIn(Pin, HIGH, 25000);  // 20 ms per + 5 ms of grace

		bool validPulse = pulseValidator(pulse);
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

		#ifdef SERVOINPUT_PIN_SPECIALIZATION
		const bool state = SERVOINPUT_DIRECT_PIN_READ(PortRegister, PinMask);
		#else
		const bool state = digitalRead(Pin);
		#endif

		if (state == HIGH) {  // rising edge
			start = micros();
		}
		else {  // falling edge
			pulseDuration = micros() - start;
			changed = true;
		}
	}

protected:
	static volatile bool changed;
	static volatile unsigned long pulseDuration;

	static unsigned long getPulseInternal() {
		// disable / enable interrupts here so the multi-byte variable is not
		// updated while it's being copied from volatile memory
		noInterrupts();
		const unsigned long pulse = ServoInputPin<Pin>::pulseDuration;
		interrupts();
		return pulse;
	}

private:
	// class instance counter, for automatic interrupt detachment
	static uint8_t refCount;

	// flag to indicate whether the class interrupt is attached, to
	// avoid making multiple calls to the platform attachment function
	static bool interruptAttached;

#ifdef SERVOINPUT_PIN_SPECIALIZATION
private:
	static SERVOINPUT_IO_REG_TYPE PinMask;  // bitmask to isolate the I/O pin
	static volatile SERVOINPUT_IO_REG_TYPE* PortRegister;  // pointer to the I/O register for the pin
#endif
};

template<uint8_t Pin> uint8_t ServoInputPin<Pin>::refCount = 0;
template<uint8_t Pin> bool ServoInputPin<Pin>::interruptAttached = false;

#ifdef SERVOINPUT_PIN_SPECIALIZATION
template<uint8_t Pin> SERVOINPUT_IO_REG_TYPE ServoInputPin<Pin>::PinMask;
template<uint8_t Pin> volatile SERVOINPUT_IO_REG_TYPE* ServoInputPin<Pin>::PortRegister;
#endif

template<uint8_t Pin> volatile bool ServoInputPin<Pin>::changed = false;
template<uint8_t Pin> volatile unsigned long ServoInputPin<Pin>::pulseDuration = 0;


class ServoInputManager {
public:
	static bool available();
	static bool allAvailable();
	static bool anyAvailable();

	static uint8_t getNumSignals();
};

extern ServoInputManager ServoInput;

// Clean up platform-specific register definitions
#undef SERVOINPUT_IO_REG_TYPE
#undef SERVOINPUT_PIN_TO_BASEREG
#undef SERVOINPUT_PIN_TO_BITMASK
#undef SERVOINPUT_DIRECT_PIN_READ
#undef SERVOINPUT_ISR_FLAG

#endif
