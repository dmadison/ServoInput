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

// This file borrows the formating and naming conventions from Paul Stoffregen's
// Encoder library. Thanks Paul!
// https://github.com/PaulStoffregen/Encoder/blob/master/utility/direct_pin_read.h

#ifndef ServoInput_Platforms_h
#define ServoInput_Platforms_h

#include <Arduino.h>


// Disable interrupt checking if platform does not support pin checks
#ifndef NOT_AN_INTERRUPT
#define SERVOINPUT_DISABLE_PIN_CHECK
#endif

// Blanket define to cover all instances
#define SERVOINPUT_PIN_SPECIALIZATION

#if defined(__AVR__) // || defined(TEENSYDUINO)

#define SERVOINPUT_IO_REG_TYPE                     uint8_t
#define SERVOINPUT_PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define SERVOINPUT_PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define SERVOINPUT_DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#elif defined(ESP8266) || defined(ESP32)

#define SERVOINPUT_IO_REG_TYPE                     uint32_t
#define SERVOINPUT_PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define SERVOINPUT_PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define SERVOINPUT_DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)
#define SERVOINPUT_ISR_FLAG                        ICACHE_RAM_ATTR

#elif defined(__SAMD21G18A__)  // Arduino MKR boards, Arm Cortex-M0 SAMD21

#define SERVOINPUT_IO_REG_TYPE                     uint32_t
#define SERVOINPUT_PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define SERVOINPUT_PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define SERVOINPUT_DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#elif defined(__IMXRT1062__)  // Teensy 4.0/4.1
#define SERVOINPUT_IO_REG_TYPE			            uint32_t
#define SERVOINPUT_PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define SERVOINPUT_PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define SERVOINPUT_DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#else  // Universal (slow) mode

#undef SERVOINPUT_PIN_SPECIALIZATION

#endif

#ifndef SERVOINPUT_ISR_FLAG
#define SERVOINPUT_ISR_FLAG
#endif


#endif
