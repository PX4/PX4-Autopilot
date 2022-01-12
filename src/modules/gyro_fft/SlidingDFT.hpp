/**
Sliding discrete Fourier transform (C++)
====

This code efficiently computes discrete Fourier transforms (DFTs) from a
continuous sequence of input values. It is a recursive algorithm that updates
the DFT when each new time-domain measurement arrives, effectively applying a
sliding window over the last *N* samples. This implementation applies the
Hanning window in order to minimise spectral leakage.

The update step has computational complexity *O(N)*. If a new DFT is required
every *M* samples, and *M* < log2(*N*), then this approach is more efficient
that recalculating the DFT from scratch each time.

This is a header-only C++ library. Simply copy sliding_dft.hpp into your
project, and use it as follows:

	// Use double precision arithmetic and a 512-length DFT
	static SlidingDFT<double, 512> dft;
	// avoid allocating on the stack because the object is large

	// When a new time sample arrives, update the DFT with:
	dft.update(x);

	// After at least 512 samples have been processed:
	std::complex<double> DC_bin = dft.dft[0];

Your application should call update() as each time domain sample arrives. Output
data is an array of `std::complex` values in the `dft` field. The length of this
array is the length of the DFT.

The output data is not valid until at least *N* samples have been processed. You
can detect this using the `is_data_valid()` method, or by storing the return
value of the `update()` method.

This is a header-only C++ library. Simply copy sliding_dft.hpp into your
project. The included CMakeLists.txt is for building the testbench.

Implementation details
----

See references [1, 2] for an overview of sliding DFT algorithms. A damping
factor is used to improve stability in the face of numerical rounding errors. If
you experience stability issues, reduce `dft.damping_factor`. It should be
slightly less than one.

Windowing is done using a Hanning window, computed in the frequency domain [1].

[1] E. Jacobsen and R. Lyons, “The Sliding DFT,” IEEE Signal Process. Mag., vol. 20, no. 2, pp. 74–80, Mar. 2003.

[2] E. Jacobsen and R. Lyons, “An Update to the Sliding DFT,” IEEE Signal Process. Mag., vol. 21, no. 1, pp. 110-111, 2004.


MIT License
----

Copyright (c) 2016 Bronson Philippa

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <math.h>

class Complex;
Complex operator*(const Complex &, const Complex &);
Complex operator/(const Complex &, const Complex &);
Complex operator-(const Complex &, const Complex &);
Complex operator-(const Complex &, const float &);

class Complex
{
public:
	Complex(float real = 0.f, float imag = 0.f) : _real(real), _imag(imag) {}
	Complex(const Complex &c) : _real(c.real()), _imag(c.imag()) {}

	float real() const { return _real; }
	float imag() const { return _imag; }

	float norm() const { return _real * _real + _imag * _imag; }

	Complex &operator= (const float &re) { _real = re; _imag = 0.f; return *this;}
	Complex &operator+=(const float &re) { _real += re; return *this;}
	Complex &operator-=(const float &re) { _real -= re; return *this;}
	Complex &operator*=(const float &re) { _real *= re; _imag *= re; return *this;}
	Complex &operator/=(const float &re) { _real /= re; _imag /= re; return *this;}

	Complex &operator= (const Complex &c)
	{
		_real = c.real();
		_imag = c.imag();
		return *this;
	}

	Complex &operator+=(const Complex &c)
	{
		_real += c.real();
		_imag += c.imag();
		return *this;
	}

	Complex &operator-=(const Complex &c)
	{
		_real -= c.real();
		_imag -= c.imag();
		return *this;
	}

	Complex &operator*=(const Complex &c)
	{
		*this = *this * Complex(c.real(), c.imag());
		return *this;
	}

	Complex &operator/=(const Complex &c)
	{
		*this = *this / Complex(c.real(), c.imag());
		return *this;
	}

private:
	float _real{0.f};
	float _imag{0.f};
};

Complex operator+(const Complex &x, const Complex &y)
{
	Complex t{x};
	t += y;
	return t;
}

Complex operator*(const Complex &z, const Complex &w)
{
	float a = z.real();
	float b = z.imag();
	float c = w.real();
	float d = w.imag();
	float ac = a * c;
	float bd = b * d;
	float ad = a * d;
	float bc = b * c;
	float x = ac - bd;
	float y = ad + bc;

	if (isnan(x) && isnan(y)) {
		bool recalc = false;

		if (isinf(a) || isinf(b)) {
			a = copysign(isinf(a) ? float(1) : float(0), a);
			b = copysign(isinf(b) ? float(1) : float(0), b);

			if (isnan(c)) {
				c = copysign(float(0), c);
			}

			if (isnan(d)) {
				d = copysign(float(0), d);
			}

			recalc = true;
		}

		if (isinf(c) || isinf(d)) {
			c = copysign(isinf(c) ? float(1) : float(0), c);
			d = copysign(isinf(d) ? float(1) : float(0), d);

			if (isnan(a)) {
				a = copysign(float(0), a);
			}

			if (isnan(b)) {
				b = copysign(float(0), b);
			}

			recalc = true;
		}

		if (!recalc && (isinf(ac) || isinf(bd) || isinf(ad) || isinf(bc))) {
			if (isnan(a)) {
				a = copysign(float(0), a);
			}

			if (isnan(b)) {
				b = copysign(float(0), b);
			}

			if (isnan(c)) {
				c = copysign(float(0), c);
			}

			if (isnan(d)) {
				d = copysign(float(0), d);
			}

			recalc = true;
		}

		if (recalc) {
			x = float(INFINITY) * (a * c - b * d);
			y = float(INFINITY) * (a * d + b * c);
		}
	}

	return Complex(x, y);
}

Complex operator-(const Complex &x, const Complex &y)
{
	Complex t(x);
	t -= y;
	return t;
}

Complex operator-(const Complex &x, const float &y)
{
	Complex t(x);
	t -= y;
	return t;
}

template <size_t DFT_Length>
class SlidingDFT
{
public:
	SlidingDFT()
	{
		// Compute the twiddle factors, and zero the x and S arrays
		for (size_t k = 0; k < DFT_Length; k++) {
			float factor = (2.f * (float)M_PI) * k / DFT_Length;

			_twiddle[k] = Complex(cosf(factor), sinf(factor));
		}
	}

	/// Determine whether the output data is valid
	bool data_valid() const { return _data_valid; }

	/// Update the calculation with a new sample
	/// Returns true if the data are valid (because enough samples have been
	/// presented), or false if the data are invalid.
	bool update(float new_x)
	{
		// Update the storage of the time domain values
		const float old_x = _x[_x_index];
		_x[_x_index] = new_x;

		// Update the DFT
		const float r = _damping_factor;
		const float r_to_N = powf(r, (float)DFT_Length);

		for (size_t k = 0; k < DFT_Length; k++) {
			_S[k] = _twiddle[k] * (r * _S[k] - r_to_N * old_x + new_x);
		}

		// Apply the Hanning window
		_dft[0] = 0.5f * _S[0] - 0.25f * (_S[DFT_Length - 1] + _S[1]);

		for (size_t k = 1; k < (DFT_Length - 1); k++) {
			_dft[k] = 0.5f * _S[k] - 0.25f * (_S[k - 1] + _S[k + 1]);
		}

		_dft[DFT_Length - 1] = 0.5f * _S[DFT_Length - 1] - 0.25f * (_S[DFT_Length - 2] + _S[0]);

		// Increment the counter
		_x_index++;

		if (_x_index >= DFT_Length) {
			_data_valid = true;
			_x_index = 0;
		}

		return _data_valid;
	}

	const auto &dft(int index) const { return _dft[index]; }

private:

	/// Frequency domain values (windowed)
	Complex _dft[DFT_Length] {};

	/// A damping factor introduced into the recursive DFT algorithm to guarantee stability.
	float _damping_factor{0.9999f};

	/// Are the frequency domain values valid? (i.e. have at elast DFT_Length data points been seen?)
	bool _data_valid{false};

	/// Time domain samples are stored in this circular buffer.
	float _x[DFT_Length] {};

	/// Index of the next item in the buffer to be used. Equivalently, the number of samples that have been seen so far modulo DFT_Length.
	size_t _x_index{0};

	/// Twiddle factors for the update algorithm
	Complex _twiddle[DFT_Length] {};

	/// Frequency domain values (unwindowed!)
	Complex _S[DFT_Length] {};
};
