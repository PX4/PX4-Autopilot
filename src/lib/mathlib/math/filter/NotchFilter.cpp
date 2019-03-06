/****************************************************************************
 *
 * @file	NotchFilter.h
 * @brief	To implement a notch filter
 * Author:  Cris.Wei <buaaxw@gmail.com>
 * Adapted  For Pixhawk4 mini
 *
 ***************************************************************************/

#include "NotchFilter.hpp"

#include <px4_defines.h>

#include <cmath>

namespace math
{

void NotchFilter::set_notch_filter(float sample_freq, float notch_freq, float notch_band, float notch_depth)
{
	if ((notch_freq <= 0.0f) || (sample_freq < 0.0f) || (notch_band < 0.0f) || (notch_depth < 0.0f)) {
		// no filtering
		_b1 = 0.0f;
		_b2 = 0.0f;

		_a0 = 0.0f;
		_a1 = 0.0f;
		_a2 = 0.0f;

		return;
	}
	_notch_freq = notch_freq;

	float a = 0.0f;
	float w = 2.0f * 3.14159f * notch_freq;

	if (w <= 0.0001f) {
		a = 1.0f / (w * w);
	}
	else {
		a = 0.0f;
	}

	float  b = notch_band / w;
	float  c = notch_depth / w;
	float  T = 1.0f / sample_freq;

	_a0 = (4.0f * a + 2.0f * c * T + T * T) / ( 4.0f * a + 2.0f * b * T + T * T);
	_a1 = (2.0f * T * T - 8.0f * a) / (4.0f * a + 2.0f * b * T + T * T);
	_a2 = (4.0f * a - 2.0f * c * T + T * T) / (4.0f * a + 2.0f * b * T + T * T);
	_b1 = (2.0f * T * T - 8.0f * a) / (4.0f * a + 2.0f * b * T + T * T);
	_b2 = (4.0f * a - 2.0f * b * T + T * T) / (4.0f * a + 2.0f * b * T + T * T);

	// reset delay elements on filter change
	_x_k2 = 0.0f;
	_x_k1 = 0.0f;
	_y_k2 = 0.0f;
	_y_k1 = 0.0f;

	return;
}

float NotchFilter::apply(float date)
{
	float x_k = date;
	float y_k = _a0 * x_k + _a1 * _x_k1 + _a2 * _x_k2 - _b1 * _y_k1 - _b2 * _y_k2;

	_x_k2 = _x_k1;
	_x_k1 = x_k;
	_y_k2 = _y_k1;
	_y_k1 = y_k;

	return y_k;
}

float NotchFilter::reset(float date)
{
	_x_k1 = date;
	_x_k2 = date;
	_y_k1 = date;
	_y_k2 = date;

	return apply(date);
}

} // namespace math