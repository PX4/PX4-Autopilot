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
 * @file tunes.h
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>
// TODO: find better way to include the number of tunes, maybe include them in the lib directly?
#include <drivers/drv_tone_alarm.h>

namespace output
{

enum NoteMode { MODE_NORMAL, MODE_LEGATO, MODE_STACCATO};

class Tunes
{
public:
	Tunes();
	Tunes(unsigned tempo, unsigned octave, unsigned note_length, NoteMode mode);
	~Tunes() = default;
	int parse_cmd(tune_control_s &tune_control, unsigned &frequency, unsigned &duration, unsigned &silence);
	int parse_string(const char *string, unsigned &frequency, unsigned &duration, unsigned &silence);

private:
	static const unsigned	_tune_max = 1024 * 8; // be reasonable about user tunes
	static const char		 *_default_tunes[TONE_NUMBER_OF_TUNES];
	static const char		 *_tune_names[TONE_NUMBER_OF_TUNES];
	static const uint8_t	_note_tab[];

	unsigned		_default_tune_number; // number of currently playing default tune (0 for none)

	const char		*_user_tune;

	const char		*_tune = nullptr;		// current tune string
	const char		*_next = nullptr;		// next note in the string

	unsigned		_tempo;
	unsigned		_note_length;
	NoteMode 		_note_mode;
	unsigned		_octave;
	unsigned		_default_tempo = 120;
	unsigned		_default_note_length = 4;
	NoteMode		_default_mode = NoteMode::MODE_NORMAL;
	unsigned		_default_octave = 4;
	// unsigned		_silence_length; // if nonzero, silence before next note
	bool			_repeat;	// if true, tune restarts at end
	int				_cbrk;	//if true, no audio output

	/**
	 * Convert note to frequency
	 * @param  note
	 * @return      frequency
	 */
	uint32_t note_to_frequency(unsigned note);

	// Calculate the duration in microseconds of play and silence for a
	// note given the current tempo, length and mode and the number of
	// dots following in the play string.
	//
	unsigned note_duration(unsigned &silence, unsigned note_length, unsigned dots);

	// Calculate the duration in microseconds of a rest corresponding to
	// a given note length.
	//
	unsigned rest_duration(unsigned rest_length, unsigned dots);

	// Parse the next note out of the string and play it
	//
	int next_note(unsigned &frequency, unsigned &duration, unsigned &silence);

	// Find the next character in the string, discard any whitespace and
	// return the canonical (uppercase) version.
	//
	int next_char();

	// Extract a number from the string, consuming all the digit characters.
	//
	unsigned next_number();

	// Consume dot characters from the string, returning the number consumed.
	//
	unsigned next_dots();

	void config_tone();

};

} /* output */
