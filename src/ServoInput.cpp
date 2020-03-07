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
	ServoInputSignal* ptr = ServoInputSignal::head;

	while (ptr != nullptr) {
		ptr->begin();
		ptr = ptr->next;
	}
}

void ServoInputManager::end() {
	ServoInputSignal* ptr = ServoInputSignal::head;

	while (ptr != nullptr) {
		ptr->end();
		ptr = ptr->next;
	}
}

boolean ServoInputManager::available() {
	return anyAvailable();
}

boolean ServoInputManager::allAvailable() {
	if (ServoInputSignal::head == nullptr) return false;  // no instances

	ServoInputSignal* ptr = ServoInputSignal::head;

	boolean ready = true;
	while (ptr != nullptr) {
		if (ptr->available() == false) {
			ready = false;
			break;  // one is false, stop iterating
		}
		ptr = ptr->next;
	}
	return ready;
}

boolean ServoInputManager::anyAvailable() {
	ServoInputSignal* ptr = ServoInputSignal::head;

	boolean ready = false;
	while (ptr != nullptr) {
		if (ptr->available() == true) {
			ready = true;
			break;  // one is true, stop iterating
		}
		ptr = ptr->next;
	}
	return ready;
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

ServoInputSignal::ServoInputSignal(uint16_t pMin, uint16_t pMax) : ServoInputSignal() {
	setRange(pMin, pMax);
}

boolean ServoInputSignal::available() const {
	const unsigned long pulse = getPulseRaw();
	return pulseValidator(pulse);
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

long ServoInputSignal::map(long outMin, long outMax) const {
	const uint16_t pulse = getPulse();
	return remap(pulse, outMin, outMax);
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
	setRangeMin(1500 - (range / 2));
	setRangeMax(1500 + (range / 2));
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

boolean ServoInputSignal::pulseValidator(unsigned long pulse) {
	return (pulse >= MinValidPulse && pulse <= MaxValidPulse);
}

long ServoInputSignal::remap(long pulse, long outMin, long outMax) const {
	if (pulse <= pulseMin) return outMin;
	if (pulse >= pulseMax) return outMax;
	return ::map(pulse, pulseMin, pulseMax, outMin, outMax);
}

