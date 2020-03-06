/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *
 */

#ifndef ServoInput_h
#define ServoInput_h

#include <Arduino.h>

class ServoInputSignal {
public:
	ServoInputSignal();
	ServoInputSignal(uint16_t pMin, uint16_t pMax);

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
