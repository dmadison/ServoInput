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

void ServoInputManager::begin() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();

	while (ptr != nullptr) {
		ptr->begin();
		ptr = ptr->getNext();
	}
}

void ServoInputManager::end() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();

	while (ptr != nullptr) {
		ptr->end();
		ptr = ptr->getNext();
	}
}

boolean ServoInputManager::available() {
	return allAvailable();
}

boolean ServoInputManager::allAvailable() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();
	boolean available = false;

	while (ptr != nullptr) {
		available = ptr->available();
		if (available == false) break;  // one not available, therefore 'all' is false
		ptr = ptr->getNext();
	}

	return available;
}

boolean ServoInputManager::anyAvailable() {
	ServoInputSignal* ptr = ServoInputSignal::getHead();
	boolean available = false;

	while (ptr != nullptr) {
		available = ptr->available();
		if (available == true) break;  // one is available, therefore 'any' is true
		ptr = ptr->getNext();
	}

	return available;
}

ServoInputManager ServoInput;  // management instance


ServoInputSignal* ServoInputSignal::head = nullptr;
ServoInputSignal* ServoInputSignal::tail = nullptr;

ServoInputSignal::ServoInputSignal() {
	// If linked list is empty, set both head and tail
	if (head == nullptr) {
		head = this;
		tail = this;
	}
	// If linked list is *not* empty, set the 'next' ptr of the tail
	// and update the tail
	else {
		tail->next = this;
		tail = this;
	}

	resetRange();  // set initial range values
}

ServoInputSignal::~ServoInputSignal() {
	// If we're at the start of the list...
	if (this == head) {
		// Option #1: Only element in the list
		if (this == tail) {
			head = nullptr;
			tail = nullptr;  // List is now empty
		}
		// Option #2: First element in the list,
		// but not *only* element
		else {
			head = next;  // Set head to next, and we're done
		}
		return;
	}

	// Option #3: Somewhere else in the list.
	// Iterate through to find it
	ServoInputSignal* ptr = head;

	while (ptr != nullptr) {
		if (ptr->next == this) {  // FOUND!
			ptr->next = this->next;  // Set the previous "next" as this entry's "next" (skip this object)
			break;  // Stop searching
		}
		ptr = ptr->next;  // Not found. Next entry...
	}

	// Option #4: Last entry in the list
	if (this == tail) {
		tail = ptr;  // Set the tail as the previous entry
	}
}

uint16_t ServoInputSignal::getPulse() const {
	const unsigned long pulse = getPulseRaw();
	if (pulseValidator(pulse) == false) return getRangeCenter();  // not valid pulse, return center
	if (pulse <= pulseMin) return pulseMin;
	if (pulse >= pulseMax) return pulseMax;
	return pulse;
}

float ServoInputSignal::getAngle() const {
	static const long ScaleFactor = 100;  // for integer to float precision
	const uint16_t pulse = getPulse();
	long out = remap(pulse, 0 * ScaleFactor, 180 * ScaleFactor);
	return (float) out / ScaleFactor;
}

float ServoInputSignal::getPercent() const {
	static const long ScaleFactor = 100;  // for integer to float precision
	const uint16_t pulse = getPulse();
	long out = remap(pulse, 0 * ScaleFactor, 100 * ScaleFactor);
	return (float) out / ScaleFactor;
}

boolean ServoInputSignal::getBoolean() const {
	const uint16_t pulse = getPulse();

	return pulse > getRangeMax() - (getRange() / 2);  // if pulse is greater than half
}

long ServoInputSignal::map(long outMin, long outMax) const {
	const uint16_t pulse = getPulse();
	return remap(pulse, outMin, outMax);
}

long ServoInputSignal::mapDeadzone(long outMin, long outMax, float zonePercent) const {
	if (abs(zonePercent) >= 1.0) return (outMin + outMax) / 2;  // deadzone >= full range, return deadzone value
	uint16_t zoneUs = getRange() * abs(zonePercent);  // convert percentage to microsecond period
	return mapDeadzonePulse(outMin, outMax, zoneUs);
}

long ServoInputSignal::mapDeadzonePulse(long outMin, long outMax, uint16_t zoneUs) const {
	const long outCenter = (outMin + outMax) / 2;  // midpoint of output values

	const uint16_t ctr = getRangeCenter();

	const uint16_t zoneHalf = zoneUs / 2;  // for symmetry around the midpoint
	const uint16_t zoneLow = ctr - zoneHalf;
	const uint16_t zoneHigh = ctr + zoneHalf;

	const uint16_t pulse = getPulse();

	long output = outCenter;  // default at center, i.e. in deadzone

	if (pulse < ctr - zoneHalf) {  // below deadzone
		output = ::map(pulse, getRangeMin(), zoneLow, outMin, outCenter);
	}
	else if (pulse > ctr + zoneHalf) {  // above deadzone
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
	setRange(PulseCenter - PulseDefaultRange, PulseCenter + PulseDefaultRange);
}

ServoInputSignal* ServoInputSignal::getHead() {
	return head;
}

ServoInputSignal* ServoInputSignal::getNext() const {
	return next;
}

boolean ServoInputSignal::pulseValidator(unsigned long pulse) {
	return pulse >= PulseCenter - PulseValidRange
		&& pulse <= PulseCenter + PulseValidRange;
}

long ServoInputSignal::remap(long pulse, long outMin, long outMax) const {
	if (pulse <= pulseMin) return outMin;
	if (pulse >= pulseMax) return outMax;
	return ::map(pulse, pulseMin, pulseMax, outMin, outMax);
}

