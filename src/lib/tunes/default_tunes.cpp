/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file default_tunes.h
 */

#include "tunes.h"

// initialise default tunes
const char *const Tunes::_default_tunes[] = {
	"", // empty to align with the index
	"MFT240L8 O4aO5dc O4aO5dc O4aO5dc L16dcdcdcdc", // startup tune
	"MBT200a8a8a8PaaaP", // ERROR tone
	"MFT200e8a8a", // Notify Positive tone
	"MFT200e8e", // Notify Neutral tone
	"MFT200e8c8e8c8e8c8", // Notify Negative tone
	"MNT75L1O2G", //arming warning
	"MBNT100a8", //battery warning slow
	"MBNT255a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8", //battery warning fast
	"MFT255L4AAAL1F#", //gps warning slow
	"MFT255L4<<<BAP", // arming failure tune
	"MFT255L16agagagag", // parachute release
	"MFT255L8ddd#d#eeff", // ekf warning
	"MFT255L4gf#fed#d", // baro warning
	"MFT100a8", // single beep
	"MFT100L4>G#6A#6B#4", // home set tune
};

// set default_tunes array size
const unsigned int Tunes::_default_tunes_size =  sizeof(_default_tunes) / sizeof(_default_tunes[0]);
