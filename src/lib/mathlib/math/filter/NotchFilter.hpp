/****************************************************************************
 *
 * @file	NotchFilter.h
 * @brief	to implement a notch filter
 * Author:  Cris.Wei <buaaxw@gmail.com>
 * Adapted  for Pixhawk4 mini
 *
 ***************************************************************************/

#pragma once

namespace math
{
class __EXPORT NotchFilter
{
public:

	NotchFilter(float sample_freq, float notch_freq, float notch_band, float notch_depth)
	{
		// set initial parameters
		set_notch_filter(sample_freq, notch_freq, notch_band, notch_depth);
	}

	// Change filter parameters
	void set_notch_filter(float sample_freq, float notch_freq, float notch_band, float notch_depth);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	float apply(float date);

	// Return the cutoff frequency
	float get_notch_freq() const { return _notch_freq; }

	// Reset the filter state to this value
	float reset(float sample);

private:

	float _notch_freq{43.0f};

	float _a0{0.0f};
	float _a1{0.0f};
	float _a2{0.0f};
	float _b1{0.0f};
	float _b2{0.0f};

	float _x_k1{0.0f}; // x-buffered sample -1
	float _x_k2{0.0f}; // x buffered sample -2
	float _y_k1{0.0f}; // y-buffered sample -1
	float _y_k2{0.0f}; // y-buffered sample -2
};

} // namespace math