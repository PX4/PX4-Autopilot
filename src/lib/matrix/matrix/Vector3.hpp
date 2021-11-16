/**
 * @file Vector3.hpp
 *
 * 3D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template <typename Type, size_t M>
class Vector;

template <typename Type>
class Dcm;

template <typename Type>
class Vector2;

template<typename Type>
class Vector3 : public Vector<Type, 3>
{
public:

	using Matrix31 = Matrix<Type, 3, 1>;

	Vector3() = default;

	Vector3(const Matrix31 &other) :
		Vector<Type, 3>(other)
	{
	}

	explicit Vector3(const Type data_[3]) :
		Vector<Type, 3>(data_)
	{
	}

	Vector3(Type x, Type y, Type z)
	{
		Vector3 &v(*this);
		v(0) = x;
		v(1) = y;
		v(2) = z;
	}

	template<size_t P, size_t Q>
	Vector3(const Slice<Type, 3, 1, P, Q> &slice_in) : Vector<Type, 3>(slice_in)
	{
	}

	template<size_t P, size_t Q>
	Vector3(const Slice<Type, 1, 3, P, Q> &slice_in) : Vector<Type, 3>(slice_in)
	{
	}

	Vector3 cross(const Matrix31 &b) const
	{
		const Vector3 &a(*this);
		return {a(1) *b(2, 0) - a(2) *b(1, 0), -a(0) *b(2, 0) + a(2) *b(0, 0), a(0) *b(1, 0) - a(1) *b(0, 0)};
	}

	/**
	 * Override matrix ops so Vector3 type is returned
	 */

	inline Vector3 operator+(Vector3 other) const
	{
		return Matrix31::operator+(other);
	}

	inline Vector3 operator+(Type scalar) const
	{
		return Matrix31::operator+(scalar);
	}

	inline Vector3 operator-(Vector3 other) const
	{
		return Matrix31::operator-(other);
	}

	inline Vector3 operator-(Type scalar) const
	{
		return Matrix31::operator-(scalar);
	}

	inline Vector3 operator-() const
	{
		return Matrix31::operator-();
	}

	inline Vector3 operator*(Type scalar) const
	{
		return Matrix31::operator*(scalar);
	}

	inline Type operator*(Vector3 b) const
	{
		return Vector<Type, 3>::operator*(b);
	}

	inline Vector3 operator%(const Matrix31 &b) const
	{
		return (*this).cross(b);
	}

	/**
	 * Override vector ops so Vector3 type is returned
	 */
	inline Vector3 unit() const
	{
		return Vector3(Vector<Type, 3>::unit());
	}

	inline Vector3 normalized() const
	{
		return unit();
	}

	const Slice<Type, 2, 1, 3, 1> xy() const
	{
		return Slice<Type, 2, 1, 3, 1>(0, 0, this);
	}

	Slice<Type, 2, 1, 3, 1> xy()
	{
		return Slice<Type, 2, 1, 3, 1>(0, 0, this);
	}


	Dcm<Type> hat() const      // inverse to Dcm.vee() operation
	{
		const Vector3 &v(*this);
		Dcm<Type> A;
		A(0, 0) = 0;
		A(0, 1) = -v(2);
		A(0, 2) = v(1);
		A(1, 0) = v(2);
		A(1, 1) = 0;
		A(1, 2) = -v(0);
		A(2, 0) = -v(1);
		A(2, 1) = v(0);
		A(2, 2) = 0;
		return A;
	}

};

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
