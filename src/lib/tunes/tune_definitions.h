/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tune_definitions.h
 *
 * Includes definition for the Tune related enum used in tunes.h and tunes.cpp.
 * The enum is used in the tune.h, which is why it needs to be defined in this separated header,
 * while other arrays are instantiated in 'tune_definitinos.cpp'
 */

#pragma once

/**
 * @brief Define TuneID enums using the 'tune_definitions.desc' file
 *
 * This is done by cleverly defining the 'PX4_DEFINE_TUNE' locally, using only the 'name' component,
 * and using it inside the definition for the enum class TuneID. Therefore the line below will end up like:
 *
 * enum class TuneID {
 *	CUSTOM,
 * 	STARTUP,
 * 	...
 * 	NONE = -1
 * };
 */
#define PX4_DEFINE_TUNE(ordinal,name,tune,interruptable) name,
enum class TuneID {
#include "tune_definitions.desc"
	STOP = 127, // Special Tune ID that stops the tune
	NONE = -1   // Undefined Tune ID
};
#undef PX4_DEFINE_TUNE
