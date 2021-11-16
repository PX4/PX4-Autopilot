/**
 * @file Slice.hpp
 *
 * A simple matrix template library.
 *
 * @author Julian Kent < julian@auterion.com >
 */

#pragma once

#include "math.hpp"


namespace matrix
{

template<typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector;

template <typename Type, size_t P, size_t Q, size_t M, size_t N>
class Slice
{
public:
	Slice(size_t x0, size_t y0, const Matrix<Type, M, N> *data) :
		_x0(x0),
		_y0(y0),
		_data(const_cast<Matrix<Type, M, N>*>(data))
	{
		static_assert(P <= M, "Slice rows bigger than backing matrix");
		static_assert(Q <= N, "Slice cols bigger than backing matrix");
		assert(x0 + P <= M);
		assert(y0 + Q <= N);
	}

	const Type &operator()(size_t i, size_t j) const
	{
		assert(i < P);
		assert(j < Q);

		return (*_data)(_x0 + i, _y0 + j);
	}

	Type &operator()(size_t i, size_t j)

	{
		assert(i < P);
		assert(j < Q);

		return (*_data)(_x0 + i, _y0 + j);
	}

	template<size_t MM, size_t NN>
	Slice<Type, P, Q, M, N> &operator=(const Slice<Type, P, Q, MM, NN> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) = other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator=(const Matrix<Type, P, Q> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) = other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator=(const Type &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) = other;
			}
		}

		return self;
	}

	// allow assigning vectors to a slice that are in the axis
	template <size_t DUMMY = 1> // make this a template function since it only exists for some instantiations
	Slice<Type, 1, Q, M, N> &operator=(const Vector<Type, Q> &other)
	{
		Slice<Type, 1, Q, M, N> &self = *this;

		for (size_t j = 0; j < Q; j++) {
			self(0, j) = other(j);
		}

		return self;
	}

	template<size_t MM, size_t NN>
	Slice<Type, P, Q, M, N> &operator+=(const Slice<Type, P, Q, MM, NN> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) += other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator+=(const Matrix<Type, P, Q> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) += other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator+=(const Type &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) += other;
			}
		}

		return self;
	}

	template<size_t MM, size_t NN>
	Slice<Type, P, Q, M, N> &operator-=(const Slice<Type, P, Q, MM, NN> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) -= other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator-=(const Matrix<Type, P, Q> &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) -= other(i, j);
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator-=(const Type &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) -= other;
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator*=(const Type &other)
	{
		Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				self(i, j) *= other;
			}
		}

		return self;
	}

	Slice<Type, P, Q, M, N> &operator/=(const Type &other)
	{
		return operator*=(Type(1) / other);
	}

	Matrix<Type, P, Q> operator*(const Type &other) const
	{
		const Slice<Type, P, Q, M, N> &self = *this;
		Matrix<Type, P, Q> res;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				res(i, j) = self(i, j) * other;
			}
		}

		return res;
	}

	Matrix<Type, P, Q> operator/(const Type &other) const
	{
		const Slice<Type, P, Q, M, N> &self = *this;
		return self * (Type(1) / other);
	}

	template<size_t R, size_t S>
	const Slice<Type, R, S, M, N> slice(size_t x0, size_t y0) const
	{
		return Slice<Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
	}

	template<size_t R, size_t S>
	Slice<Type, R, S, M, N> slice(size_t x0, size_t y0)
	{
		return Slice<Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
	}

	void copyTo(Type dst[P * Q]) const
	{
		const Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				dst[i * N + j] = self(i, j);
			}
		}
	}

	void copyToColumnMajor(Type dst[P * Q]) const
	{
		const Slice<Type, P, Q, M, N> &self = *this;

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				dst[i + (j * M)] = self(i, j);
			}
		}
	}

	Vector < Type, P < Q ? P : Q > diag() const
	{
		const Slice<Type, P, Q, M, N> &self = *this;
		Vector < Type, P < Q ? P : Q > res;

		for (size_t j = 0; j < (P < Q ? P : Q); j++) {
			res(j) = self(j, j);
		}

		return res;
	}

	Type norm_squared() const
	{
		const Slice<Type, P, Q, M, N> &self = *this;
		Type accum(0);

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				accum += self(i, j) * self(i, j);
			}
		}

		return accum;
	}

	Type norm() const
	{
		return matrix::sqrt(norm_squared());
	}

	bool longerThan(Type testVal) const
	{
		return norm_squared() > testVal * testVal;
	}

	Type max() const
	{
		Type max_val = (*this)(0, 0);

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				Type val = (*this)(i, j);

				if (val > max_val) {
					max_val = val;
				}
			}
		}

		return max_val;
	}

	Type min() const
	{
		Type min_val = (*this)(0, 0);

		for (size_t i = 0; i < P; i++) {
			for (size_t j = 0; j < Q; j++) {
				Type val = (*this)(i, j);

				if (val < min_val) {
					min_val = val;
				}
			}
		}

		return min_val;
	}

private:
	size_t _x0, _y0;
	Matrix<Type, M, N> *_data;
};

}
