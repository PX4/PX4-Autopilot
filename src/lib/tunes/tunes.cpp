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
 * @file tunes.cpp
 */

#include "tunes.h"

#include <px4_log.h>
#include <math.h>
#include <ctype.h>

#define TUNE_ERROR -1
#define TUNE_STOP 0
#define TUNE_CONTINUE 1

#define BEAT_TIME_CONVERSION_MS (60 * 1000 * 4)
#define BEAT_TIME_CONVERSION_US BEAT_TIME_CONVERSION_MS * 1000
#define BEAT_TIME_CONVERSION BEAT_TIME_CONVERSION_US

using namespace output;

// initialise default tunes
const char *Tunes::_default_tunes[] = {
	"", // empty to align with the index
	"MFT240L8 O2aO3dc O2aO3dc O2aO3dc L16dcdcdcdc", // startup tune
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

// semitone offsets from C for the characters 'A'-'G'
const uint8_t Tunes::_note_tab[] = {9, 11, 0, 2, 4, 5, 7};

Tunes::Tunes(unsigned default_tempo, unsigned default_octave, unsigned default_note_length,
	     NoteMode default_mode):
	_default_tempo(default_tempo),
	_default_note_length(default_note_length),
	_default_mode(default_mode),
	_default_octave(default_octave)
{
	config_tone();
}

Tunes::Tunes(): Tunes(120, 4, 4, NoteMode::MODE_NORMAL)
{
}

void Tunes::config_tone()
{
	// reset pointer
	_tune = nullptr;
	_next = nullptr;
	// reset music parameter
	_tempo = _default_tempo;
	_note_length = _default_note_length;
	_note_mode = _default_mode;
	_octave = _default_octave;
}

int Tunes::parse_cmd(struct tune_control_s &tune_control, unsigned &frequency, unsigned &duration, unsigned &silence)
{
	int continue_sequnece = 0;

	switch (tune_control.tune_id) {
	case tune_control_s::STARTUP:
	case tune_control_s::ERROR:
	case tune_control_s::NOTIFY_POSITIVE:
	case tune_control_s::NOTIFY_NEUTRAL:
	case tune_control_s::NOTIFY_NEGATIVE:
	case tune_control_s::ARMING_WARNING:
	case tune_control_s::BATTERY_WARNING_SLOW:
	case tune_control_s::BATTERY_WARNING_FAST:
	case tune_control_s::GPS_WARNING:
	case tune_control_s::PARACHUTE_RELEASE:
	case tune_control_s::EKF_WARNING:
	case tune_control_s::BARO_WARNING:
	case tune_control_s::SINGLE_BEEP:
	case tune_control_s::HOME_SET:

		// set tune string the first time
		if (_tune == nullptr) {
			_tune = _default_tunes[tune_control.tune_id];
			_next = _tune;
		}

		continue_sequnece = next_note(frequency, duration, silence);
		break;

	case tune_control_s::CUSTOM:
	default:
		frequency = (unsigned)tune_control.frequency;
		duration = (unsigned)tune_control.duration;
		silence = 0;
		break;
	}

	return continue_sequnece;
}

int Tunes::parse_string(const char *string, unsigned &frequency, unsigned &duration, unsigned &silence)
{
	// set tune string the first time
	if (_tune == nullptr) {
		_tune = string;
		_next = _tune;
	}

	return next_note(frequency, duration, silence);
}

unsigned Tunes::note_to_frequency(unsigned note)
{
	// compute the frequency (Hz)
	return (unsigned)(880.0f * powf(2.0f, ((int)note - 46) / 12.0f));
}

unsigned Tunes::note_duration(unsigned &silence, unsigned note_length, unsigned dots)
{
	unsigned whole_note_period = BEAT_TIME_CONVERSION / _tempo;

	if (note_length == 0) {
		note_length = 1;
	}

	unsigned note_period = whole_note_period / note_length;

	switch (_note_mode) {
	case MODE_NORMAL:
		silence = note_period / 8;
		break;

	case MODE_STACCATO:
		silence = note_period / 4;
		break;

	default:
	case MODE_LEGATO:
		silence = 0;
		break;
	}

	note_period -= silence;

	unsigned dot_extension = note_period / 2;

	while (dots--) {
		note_period += dot_extension;
		dot_extension /= 2;
	}

	return note_period;
}

unsigned Tunes::rest_duration(unsigned rest_length, unsigned dots)
{
	unsigned whole_note_period = BEAT_TIME_CONVERSION / _tempo;

	if (rest_length == 0) {
		rest_length = 1;
	}

	unsigned rest_period = whole_note_period / rest_length;

	unsigned dot_extension = rest_period / 2;

	while (dots--) {
		rest_period += dot_extension;
		dot_extension /= 2;
	}

	return rest_period;
}

int Tunes::next_char()
{
	while (isspace(*_next)) {
		_next++;
	}

	return toupper(*_next);
}

unsigned Tunes::next_number()
{
	unsigned number = 0;
	int c;

	for (;;) {
		c = next_char();

		if (!isdigit(c)) {
			return number;
		}

		_next++;
		number = (number * 10) + (c - '0');
	}
}

unsigned Tunes::next_dots()
{
	unsigned dots = 0;

	while (next_char() == '.') {
		_next++;
		dots++;
	}

	return dots;
}

int Tunes::next_note(unsigned &frequency, unsigned &duration, unsigned &silence)
{
	// make sure we still have a tune - may be removed by the write / ioctl handler
	if ((_next == nullptr) || (_tune == nullptr)) {
		return TUNE_ERROR;
	}

	// parse characters out of the string until we have resolved a note
	unsigned note = 0;
	unsigned note_length = _note_length;

	// NOTE: is this while loop necessary?
	while (note == 0) {
		// we always need at least one character from the string
		int c = next_char();

		if (c == 0) {
			goto tune_end;
		}

		_next++;

		switch (c) {
		case 'L':	// select note length
			_note_length = next_number();

			if (_note_length < 1) {
				goto tune_error;
			}

			break;

		case 'O':	// select octave
			_octave = next_number();

			if (_octave > 6) {
				_octave = 6;
			}

			break;

		case '<':	// decrease octave
			if (_octave > 0) {
				_octave--;
			}

			break;

		case '>':	// increase octave
			if (_octave < 6) {
				_octave++;
			}

			break;

		case 'M':	// select inter-note gap
			c = next_char();

			if (c == 0) {
				goto tune_error;
			}

			_next++;

			switch (c) {
			case 'N':
				_note_mode = MODE_NORMAL;
				break;

			case 'L':
				_note_mode = MODE_LEGATO;
				break;

			case 'S':
				_note_mode = MODE_STACCATO;
				break;

			case 'F':
				_repeat = false;
				break;

			case 'B':
				_repeat = true;
				break;

			default:
				goto tune_error;
			}

			break;

		case 'P':	// pause for a note length
			frequency = 0;
			duration = 0;
			silence = rest_duration(next_number(), next_dots());
			return TUNE_CONTINUE;

		case 'T': {	// change tempo
				unsigned nt = next_number();

				if ((nt >= 32) && (nt <= 255)) {
					_tempo = nt;

				} else {
					goto tune_error;
				}

				break;
			}

		case 'N':	// play an arbitrary note
			note = next_number();

			if (note > 84) {
				goto tune_error;
			}

			if (note == 0) {
				// this is a rest - pause for the current note length
				silence = rest_duration(_note_length, next_dots());
				return TUNE_CONTINUE;
			}

			break;

		case 'A'...'G':	// play a note in the current octave
			note = _note_tab[c - 'A'] + (_octave * 12) + 1;
			c = next_char();

			switch (c) {
			case '#':	// up a semitone
			case '+':
				if (note < 84) {
					note++;
				}

				_next++;
				break;

			case '-':	// down a semitone
				if (note > 1) {
					note--;
				}

				_next++;
				break;

			default:
				// 0 / no next char here is OK
				break;
			}

			// shorthand length notation
			note_length = next_number();

			if (note_length == 0) {
				note_length = _note_length;
			}

			break;

		default:
			goto tune_error;
		}
	}

	// compute the duration of the note and the following silence (if any)
	duration = note_duration(silence, note_length, next_dots());

	// compute the note frequency
	frequency = note_to_frequency(note);

	return TUNE_CONTINUE;

	// tune looks bad (unexpected EOF, bad character, etc.)
tune_error:
	// syslog(LOG_ERR, "tune error\n");
	_repeat = false;		// don't loop on error
	config_tone();
	return TUNE_ERROR;
	// stop (and potentially restart) the tune
tune_end:
	// restore intial parameter
	config_tone();

	if (_repeat) {
		return TUNE_CONTINUE;

	} else {
		_default_tune_number = 0;
		return TUNE_STOP;
	}
}
