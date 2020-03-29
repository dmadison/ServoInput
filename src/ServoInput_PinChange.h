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

#ifndef ServoInput_PinChange_h
#define ServoInput_PinChange_h

// Checks if the currently selected board is supported by the PinChangeInterrupt library

// Arduino Uno: ATmega328,   ATmega328P, ATmega168, or ATmega88
// Arduino Mega / Mega2560:  ATmega2560, AVR_ATmega1280, or AVR_ATmega640
// Arduino Leonardo / Micro: ATmega32U4, ATmega16U4
// U2 Series (Hoodloader2):  AT90USB82, AT90USB162, ATmega32U2, AVR_ATmega16U2, AVR_ATmega8U2
// ATtiny series x5:         ATtiny25, ATtiny45, ATtiny85
// ATtiny 13A:               ATtiny13
// ATtiny series x4:         ATtiny24, ATtiny44, ATtiny84
// 1284p and 644p:           ATmega1284P, ATmega644P
// ATtiny x41:               ATtinyX41, ATtiny441, ATtiny841
// ATtiny x313:              ATtinyX313

#ifdef ARDUINO_ARCH_AVR  // only AVR architecture is supported

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega88__) || \
	defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__) || \
	defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) || \
	defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__) || \
	defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || \
	defined(__AVR_ATtiny13__) || \
	defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || \
	defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) || \
	defined(__AVR_ATtinyX41__) || defined(__AVR_ATtiny441__) || defined(__AVR_ATtiny841__) || \
	defined(__AVR_ATtinyX313__)
	
#include <PinChangeInterrupt.h>  // board is compatible! import NicoHood's PinChangeInterrupt library
#define SERVOINPUT_USING_PCINTLIB

#endif  // boards check
#endif  // architecture check

#endif  // include guard
