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
 * @file tune_definitions.cpp
 *
 * Includes definitions for the arrays defined in the scope of Tunes class library
 */

#include "tunes.h"

/**
 * @brief Define default tune MML strings
 *
 * It will generate the list of MML strings somewhat like:
 *
 * const char *const Tunes::_default_tunes[] = {
 * 	"",
 * 	"MFT240L8 O4aO5dc O4aO5dc O4aO5dc L16dcdcdcdc",
 * 	...
 * };
 */
#define PX4_DEFINE_TUNE(ordinal,name,tune,interruptable) tune,
const char *const Tunes::_default_tunes[] = {
#include "tune_definitions.desc"
};
#undef PX4_DEFINE_TUNE

/**
 * @brief Define a bool list indicating if each default tunes are interruptable
 *
 * It will be somewhat like:
 *
 * const bool Tunes::_default_tunes_interruptable[] = {
 *	true,
 *	true,
 * 	...
 * };
 */
#define PX4_DEFINE_TUNE(ordinal,name,tune,interruptable) interruptable,
const bool Tunes::_default_tunes_interruptable[] = {
#include "tune_definitions.desc"
};
#undef PX4_DEFINE_TUNE

// Set the default_tunes array size
const unsigned int Tunes::_default_tunes_size =  sizeof(_default_tunes) / sizeof(_default_tunes[0]);
