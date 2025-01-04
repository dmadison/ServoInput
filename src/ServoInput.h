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


/**
* Servo signal input base class
*/
class ServoInputSignal {
public:
	/** Class constructor */
	ServoInputSignal();

	/** Class destructor */
	virtual ~ServoInputSignal();

	/** Attach the interrupt function */
	virtual void attach() = 0;

	/** Detach the interrupt function */
	virtual void detach() = 0;

	/**
	* Check if new data is available
	*
	* Data is marked as 'available' when a new signal has been read
	* and is within the valid range.
	*
	* @returns 'true' if a new signal has been read, 'false' otherwise
	*/
	virtual bool available() const = 0;


	/**
	* Gets the pulse duration, filtered
	*
	* If the pulse duration is outside the expected valid range, the previous
	* pulse value is returned.
	* 
	* If the pulse duration is outside the user-set range, it is constrained
	* to the user-set range limits.
	*
	* @returns the pulse duration in microseconds, filtered
	*/
	uint16_t getPulse();

	/**
	* Gets the pulse duration, unfiltered
	*
	* @returns the pulse duration in microseconds, unfiltered
	*/
	virtual unsigned long getPulseRaw() const = 0;

	/**
	* Gets the angle of the virtual servo
	*
	* @note This does not measure the angle of the connected servo, only the
	*       angle commanded by the signal.
	*
	* @returns angle of the servo, 0.0 - 180.0 degrees
	*/
	float getAngle();

	/**
	* Gets the travel percentage of the virtual servo
	*
	* @note This does not measure the travel of the connected servo, only the
	*       position commanded by the signal.
	*
	* @returns travel percentage, 0.0 - 1.0
	*/
	float getPercent();

	/**
	* Gets the virtual servo position as a boolean
	*
	* This is useful for reading switches from RC transmitters.
	*
	* @returns 0 if the position is in the bottom of the range, 1 if
	*          the position is in the top of the range
	*/
	bool getBoolean();

	/**
	* Maps the virtual servo position to a range
	*
	* @param outMin the minimum value of the range to map to
	* @param outMax the maximum value of the range to map to
	*
	* @returns the virtual servo position, mapped to a range
	*/
	long map(long outMin, long outMax);

	/**
	* Maps the virtual servo position to a range, with a deadzone percentage
	*
	* This maps the servo position to a range, but applies a deadzone
	* to the center. If the servo position is within the deadzone, the
	* center of the output range is returned.
	*
	* @param outMin      the minimum value of the range to map to
	* @param outMax      the maximum value of the range to map to
	* @param zonePercent the deadzone percentage to use, from 0.0 - 1.0
	*
	* @returns the virtual servo position, mapped to a range with a deadzone
	*/
	long mapDeadzone(long outMin, long outMax, float zonePercent);

	/**
	* Maps the virtual servo position to a range, with a deadzone duration
	*
	* This maps the servo position to a range, but applies a deadzone
	* to the center. If the servo position is within the deadzone, the
	* center of the output range is returned.
	*
	* @param outMin      the minimum value of the range to map to
	* @param outMax      the maximum value of the range to map to
	* @param zoneUs      the deadzone duration, in microseconds
	*
	* @returns the virtual servo position, mapped to a range with a deadzone
	*/
	long mapDeadzonePulse(long outMin, long outMax, uint16_t zoneUs);


	/**
	* Gets the filtering range
	*
	* @returns the range, in microseconds
	*/
	uint16_t getRange() const;

	/**
	* Gets the filtering range minimum
	*
	* @returns the filtering range minimum pulse duration, in microseconds
	*/
	uint16_t getRangeMin() const;

	/**
	* Gets the filtering range maximum
	*
	* @returns the filtering range maximum pulse duration, in microseconds
	*/
	uint16_t getRangeMax() const;

	/**
	* Gets the center of the filtering range
	*
	* @returns the center of the filtering range, in microseconds
	*/
	uint16_t getRangeCenter() const;

	/**
	* Sets the filtering range
	*
	* This range is used to filter the output from the
	* ServoInputSignal::getPulse() function.  It is also used as the input bounds
	* for calculating the servo's angle, travel percentage, and mapped output.
	*
	* When using this function, the range is centered at 1500 us.
	*
	* @param range the total range to set, in microseconds
	*/
	void setRange(uint16_t range);

	/**
	* Sets the filtering range
	*
	* This range is used to filter the output from the
	* `ServoInputSignal::getPulse()` function.  It is also used as the input bounds
	* for calculating the servo's angle, travel percentage, and mapped output.
	*
	* @param min the minimum pulse duration, in microseconds
	* @param max the maximum pulse duration, in microseconds
	*/
	void setRange(uint16_t min, uint16_t max);

	/**
	* Sets the filtering range minimum
	*
	* @param min the range minimum pulse duration, in microseconds
	*/
	void setRangeMin(uint16_t min);

	/**
	* Sets the filtering range maximum
	*
	* @param max the range maximum pulse duration, in microseconds
	*/
	void setRangeMax(uint16_t max);

	/**
	* Resets the filtering range to its default values
	*
	* By default, the range is 1000 us, centered at 1500 us (1000 - 2000 us).
	*/
	void resetRange();


	/**
	* Get the pin number
	*
	* @returns the Arduino pin number used by this signal
	*/
	virtual uint8_t getPin() const = 0;


	/**
	* Get the head of the linked list
	*
	* @returns pointer to the head of the linked list
	*/
	static ServoInputSignal* getHead();

	/**
	* Get the next item in the linked list
	*
	* @returns pointer to the next object in the linked list
	*/
	ServoInputSignal* getNext() const;

protected:
	/** The center of the pulse range, in microseconds */
	static const uint16_t PulseCenter = 1500;

	/** The validator pulse range, in microseconds (500 - 2500) */
	static const uint16_t PulseValidRange = 2000;

	/** The default filtering pulse range, in microseconds (1000 - 2000) */
	static const uint16_t PulseDefaultRange = 1000;

	/**
	* Checks if a pulse is within a valid range
	*
	* This is a sanity check to make sure that read pulses are within the
	* expected range for a servo control signal.
	*
	* @param pulse the pulse duration to validate
	* @returns 'true' if the pulse is valid, 'false' otherwise
	*/
	static bool pulseValidator(unsigned long pulse);


	/**
	* Remap the pulse duration to a new range
	*
	* This incorporates the filtering range set in
	* `ServoInputSignal::setRange(uint16_t, uint16_t)`.
	*
	* @param pulse  the pulse duration
	* @param outMin the minimum value of the range to map to
	* @param outMax the maximum value of the range to map to
	*/
	long remap(long pulse, long outMin, long outMax) const;

private:
	static ServoInputSignal* head;     ///< head of the linked list
	ServoInputSignal* next = nullptr;  ///< next object in the linked list

	uint16_t pulseMin;                 ///< filtering range minimum value
	uint16_t pulseMax;                 ///< filtering range maximum value

	uint16_t lastPulse = 0;            ///< the last valid pulse duration
};


/**
* Servo signal input template class
*
* @tparam Pin the Arduino pin number to use
*/
template<uint8_t Pin>
class ServoInputPin : public ServoInputSignal {
public:
	/** Class constructor, default */
	ServoInputPin() {
		#ifdef SERVOINPUT_PIN_SPECIALIZATION
			ServoInputPin<Pin>::PinMask = SERVOINPUT_PIN_TO_BITMASK(Pin);
			ServoInputPin<Pin>::PortRegister = SERVOINPUT_PIN_TO_BASEREG(Pin);
		#endif
		pinMode(Pin, INPUT_PULLUP);
		
		ServoInputPin<Pin>::refCount++;
	}

	/**
	* Class constructor, ranged
	*
	* @param pMin pulse minimum duration, for filter range
	* @param pMax pulse maximum duration, for filter range
	*
	* @see ServoInputSignal::setRange(uint16_t min, uint16_t max)
	*/
	ServoInputPin(uint16_t pMin, uint16_t pMax) : ServoInputPin() {
		ServoInputSignal::setRange(pMin, pMax);
	}

	/** Class destructor */
	~ServoInputPin() {
		ServoInputPin<Pin>::refCount--;
		if (ServoInputPin<Pin>::refCount == 0) {
			this->detach();  // no more class instances, detach interrupt
		}
	}

	/** @copydoc ServoInputSignal::attach() */
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

	/** @copydoc ServoInputSignal::detach() */
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

	/** @copydoc ServoInputSignal::available() */
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

	/**
	* Perform a blocking read of the pin
	*
	* @warning This function will pause code execution for up to 25 milliseconds.
	*          It is not recommended to use this unless the interrupt-based
	*          functionality of the library cannot be used for some reason.
	*
	* @returns 'true' if data was read, 'false' otherwise
	*/
	bool read() {
		unsigned long pulse = pulseIn(Pin, HIGH, 25000);  // 20 ms per + 5 ms of grace

		bool validPulse = pulseValidator(pulse);
		if (validPulse == true) {
			pulseDuration = pulse;  // pulse is valid, store result
		}
		return validPulse;
	}

	/** @copydoc ServoInputSignal::getPulseRaw() const */
	unsigned long getPulseRaw() const {
		const unsigned long pulse = getPulseInternal();
		ServoInputPin<Pin>::changed = false;  // value has been read, is not longer 'new'
		return pulse;
	}

	/** @copydoc ServoInputSignal::getPin() const */
	uint8_t getPin() const {
		return Pin;
	}

	/**
	* Interrupt service routine
	*
	* @warning Do not call this function in user-code unless you are
	*          implementing your own interrupt handling. Calling it arbitrarily
	*          will prevent signal measurement from working properly.
	*/
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
	/** Flag to indicate that pulse duration has changed */
	static volatile bool changed;

	/** The recorded raw pulse duration */
	static volatile unsigned long pulseDuration;

	/**
	* Read the raw pulse duration, internally
	*
	* This is a separate function from `getPulseRaw()` because we need to disable
	* interrupts momentarily so that the value is not updated by the ISR while
	* the multi-byte value is being read from memory on 8-bit systems.
	*/
	static unsigned long getPulseInternal() {
		// disable / enable interrupts here so the multi-byte variable is not
		// updated while it's being copied from volatile memory
		noInterrupts();
		const unsigned long pulse = ServoInputPin<Pin>::pulseDuration;
		interrupts();
		return pulse;
	}

private:
	/** class instance counter, for automatic interrupt detachment */
	static uint8_t refCount;

	/**
	* flag to indicate whether the class interrupt is attached, to
	* avoid making multiple calls to the platform attachment function
	*/
	static bool interruptAttached;

#ifdef SERVOINPUT_PIN_SPECIALIZATION
private:
	/** bitmask to isolate the I/O pin */
	static SERVOINPUT_IO_REG_TYPE PinMask;

	/** pointer to the I/O register for the pin */
	static volatile SERVOINPUT_IO_REG_TYPE* PortRegister;
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


/**
* ServoInput linked list management class
*/
class ServoInputManager {
public:
	/**
	* Attach the interrupts for all signals
	* @see ServoInputSignal::attach()
	*/
	static void attach();

	/**
	* Detach the interrupts for all signals
	* @see ServoInputSignal::detach()
	*/
	static void detach();

	/**
	* Check if data is available on all pins
	* @see ServoInputManager::allAvailable()
	*/
	static bool available();

	/**
	* Check if data is available on all pins
	* @see ServoInputSignal::available()
	*/
	static bool allAvailable();

	/**
	* Check if data is available on any pins
	* @see ServoInputSignal::available()
	*/
	static bool anyAvailable();

	/**
	* Get the number of signals
	*
	* @returns the number of `ServoInputSignal` objects in the linked list
	*/
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
