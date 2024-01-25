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

#include "ServoInput.h"

bool ServoInputManager::available() {
	return allAvailable();
}

bool ServoInputManager::allAvailable() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();
	bool available = false;

	while (ptr != nullptr) {
		available = ptr->available();
		if (available == false) break;  // one not available, therefore 'all' is false
		ptr = ptr->getNext();
	}

	return available;
}

bool ServoInputManager::anyAvailable() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();
	bool available = false;

	while (ptr != nullptr) {
		available = ptr->available();
		if (available == true) break;  // one is available, therefore 'any' is true
		ptr = ptr->getNext();
	}

	return available;
}

uint8_t ServoInputManager::getNumSignals() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();

	uint8_t n = 0;
	while (ptr != nullptr) {
		n++;
		ptr = ptr->getNext();
	}

	return n;
}

ServoInputManager ServoInput;  // management instance


ServoInputSignal* ServoInputSignal::head = nullptr;

ServoInputSignal::ServoInputSignal() {
	// If linked list is empty, set the head
	if (head == nullptr) {
		head = this;
	}
	// If linked list is *not* empty, set the 'next' ptr of the
	// last entry in the list
	else {
		ServoInputSignal* last = head;
		while (true) {
			if (last->next == nullptr) break;  // found last entry
			last = last->next;
		}
		last->next = this;
	}

	resetRange();  // set initial range values
}

ServoInputSignal::~ServoInputSignal() {
	// If we're at the start of the list...
	if (this == head) {
		head = next;  // Set head to next, and we're done
		return;
	}

	// Otherwise we're somewhere else in the list. Iterate through to find it.
	ServoInputSignal* ptr = head;

	while (ptr != nullptr) {
		if (ptr->next == this) {  // FOUND!
			ptr->next = this->next;  // Set the previous "next" as this entry's "next" (skip this object)
			break;  // Stop searching
		}
		ptr = ptr->next;  // Not found. Next entry...
	}
}

uint16_t ServoInputSignal::getPulse() {
	unsigned long pulse = getPulseRaw();
	if (pulseValidator(pulse) == false) pulse = lastPulse;  // not valid pulse, use last

	if      (pulse < pulseMin) pulse = pulseMin;
	else if (pulse > pulseMax) pulse = pulseMax;
	lastPulse = (uint16_t) pulse;  // buffer pulse 

	return lastPulse;
}

float ServoInputSignal::getAngle() {
	static const long ScaleFactor = 100;  // for integer to float precision
	const uint16_t pulse = getPulse();
	long out = remap(pulse, 0 * ScaleFactor, 180 * ScaleFactor);
	return (float) out / ScaleFactor;
}

float ServoInputSignal::getPercent() {
	static const long ScaleFactor = 10000;  // for integer to float precision
	const uint16_t pulse = getPulse();
	long out = remap(pulse, 0 * ScaleFactor, 1 * ScaleFactor);
	return (float) out / ScaleFactor;
}

bool ServoInputSignal::getBoolean() {
	const uint16_t pulse = getPulse();
	return pulse > getRangeCenter();
}

long ServoInputSignal::map(long outMin, long outMax) {
	const uint16_t pulse = getPulse();
	return remap(pulse, outMin, outMax);
}

long ServoInputSignal::mapDeadzone(long outMin, long outMax, float zonePercent) {
	if (abs(zonePercent) >= 1.0) return (outMin + outMax) / 2;  // deadzone >= full range, return deadzone value
	uint16_t zoneUs = getRange() * abs(zonePercent);  // convert percentage to microsecond period
	return mapDeadzonePulse(outMin, outMax, zoneUs);
}

long ServoInputSignal::mapDeadzonePulse(long outMin, long outMax, uint16_t zoneUs) {
	const long outCenter = (outMin + outMax) / 2;  // midpoint of output values
	if (zoneUs > getRange()) return outCenter;  // if deadzone bigger than range, we must be in it

	const uint16_t ctr = getRangeCenter();

	const uint16_t zoneHalf = zoneUs / 2;  // for symmetry around the midpoint
	const uint16_t zoneLow = ctr - zoneHalf;
	const uint16_t zoneHigh = ctr + zoneHalf;

	const uint16_t pulse = getPulse();

	long output = outCenter;  // default at center, i.e. in deadzone

	if (pulse < zoneLow) {  // below deadzone
		output = ::map(pulse, getRangeMin(), zoneLow, outMin, outCenter);
	}
	else if (pulse > zoneHigh) {  // above deadzone
		output = ::map(pulse, zoneHigh, getRangeMax(), outCenter, outMax);
	}

	return output;
}

uint16_t ServoInputSignal::getRange() const {
	return pulseMax - pulseMin;
}

uint16_t ServoInputSignal::getRangeMin() const {
	return pulseMin;
}

uint16_t ServoInputSignal::getRangeMax() const {
	return pulseMax;
}

uint16_t ServoInputSignal::getRangeCenter() const {
	return ((pulseMax - pulseMin) / 2) + pulseMin;  // 1500 us with default min/max
}

void ServoInputSignal::setRange(uint16_t range) {
	setRangeMin(PulseCenter - (range / 2));
	setRangeMax(PulseCenter + (range / 2));
}

void ServoInputSignal::setRange(uint16_t min, uint16_t max) {
	setRangeMin(min);
	setRangeMax(max);
}

void ServoInputSignal::setRangeMin(uint16_t min) {
	if(pulseValidator(min)) pulseMin = min;
}

void ServoInputSignal::setRangeMax(uint16_t max) {
	if (pulseValidator(max)) pulseMax = max;
}

void ServoInputSignal::resetRange() {
	setRange(PulseDefaultRange);
}

ServoInputSignal* ServoInputSignal::getHead() {
	return head;
}

ServoInputSignal* ServoInputSignal::getNext() const {
	return next;
}

bool ServoInputSignal::pulseValidator(unsigned long pulse) {
	return pulse >= PulseCenter - (PulseValidRange / 2)
		&& pulse <= PulseCenter + (PulseValidRange / 2);
}

long ServoInputSignal::remap(long pulse, long outMin, long outMax) const {
	if (pulse <= pulseMin) return outMin;
	if (pulse >= pulseMax) return outMax;
	return ::map(pulse, pulseMin, pulseMax, outMin, outMax);
}

