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

ServoInputSignal::ServoInputSignal() {}

ServoInputSignal::ServoInputSignal(uint16_t pMin, uint16_t pMax) {
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

