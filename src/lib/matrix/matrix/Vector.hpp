/**
 * @file Vector.hpp
 *
 * Vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector : public Matrix<Type, M, 1>
{
public:
	using MatrixM1 = Matrix<Type, M, 1>;

	Vector() = default;

	Vector(const MatrixM1 &other) :
		MatrixM1(other)
	{
	}

	explicit Vector(const Type data_[M]) :
		MatrixM1(data_)
	{
	}

	template<size_t P, size_t Q>
	Vector(const Slice<Type, M, 1, P, Q> &slice_in) :
		Matrix<Type, M, 1>(slice_in)
	{
	}

	template<size_t P, size_t Q, size_t DUMMY = 1>
	Vector(const Slice<Type, 1, M, P, Q> &slice_in)
	{
		Vector &self(*this);

		for (size_t i = 0; i < M; i++) {
			self(i) = slice_in(0, i);
		}
	}

	inline const Type &operator()(size_t i) const
	{
		assert(i < M);

		const MatrixM1 &v = *this;
		return v(i, 0);
	}

	inline Type &operator()(size_t i)
	{
		assert(i < M);

		MatrixM1 &v = *this;
		return v(i, 0);
	}

	Type dot(const MatrixM1 &b) const
	{
		const Vector &a(*this);
		Type r(0);

		for (size_t i = 0; i < M; i++) {
			r += a(i) * b(i, 0);
		}

		return r;
	}

	inline Type operator*(const MatrixM1 &b) const
	{
		const Vector &a(*this);
		return a.dot(b);
	}

	inline Vector operator*(Type b) const
	{
		return Vector(MatrixM1::operator*(b));
	}

	Type norm() const
	{
		const Vector &a(*this);
		return Type(matrix::sqrt(a.dot(a)));
	}

	Type norm_squared() const
	{
		const Vector &a(*this);
		return a.dot(a);
	}

	inline Type length() const
	{
		return norm();
	}

	inline void normalize()
	{
		(*this) /= norm();
	}

	Vector unit() const
	{
		return (*this) / norm();
	}

	Vector unit_or_zero(const Type eps = Type(1e-5)) const
	{
		const Type n = norm();

		if (n > eps) {
			return (*this) / n;
		}

		return Vector();
	}

	inline Vector normalized() const
	{
		return unit();
	}

	bool longerThan(Type testVal) const
	{
		return norm_squared() > testVal * testVal;
	}

	Vector sqrt() const
	{
		const Vector &a(*this);
		Vector r;

		for (size_t i = 0; i < M; i++) {
			r(i) = Type(matrix::sqrt(a(i)));
		}

		return r;
	}
};

} // namespace matrix
