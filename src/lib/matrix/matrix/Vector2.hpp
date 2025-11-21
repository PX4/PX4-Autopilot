/**
 * @file Vector2.hpp
 *
 * 2D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "Vector.hpp"

namespace matrix
{

template<typename Type>
class Vector2 : public Vector<Type, 2>
{
public:

	using Matrix21 = Matrix<Type, 2, 1>;
	using Vector3 = Vector<Type, 3>;

	Vector2() = default;

	Vector2(const Matrix21 &other) :
		Vector<Type, 2>(other)
	{
	}

	explicit Vector2(const Type data_[2]) :
		Vector<Type, 2>(data_)
	{
	}

	Vector2(Type x, Type y)
	{
		Vector2 &v(*this);
		v(0) = x;
		v(1) = y;
	}

	using base = Vector<Type, 2>;
	using base::base;

	explicit Vector2(const Vector3 &other)
	{
		Vector2 &v(*this);
		v(0) = other(0);
		v(1) = other(1);
	}

	Type cross(const Matrix21 &b) const
	{
		const Vector2 &a(*this);
		return a(0) * b(1, 0) - a(1) * b(0, 0);
	}

	/**
	 * Override matrix ops so Vector2 type is returned
	 */

	Vector2 operator+(Vector2 other) const
	{
		return Matrix21::operator+(other);
	}

	Vector2 operator+(Type scalar) const
	{
		return Matrix21::operator+(scalar);
	}

	Vector2 operator-(Vector2 other) const
	{
		return Matrix21::operator-(other);
	}

	Vector2 operator-(Type scalar) const
	{
		return Matrix21::operator-(scalar);
	}

	Vector2 operator-() const
	{
		return Matrix21::operator-();
	}

	Vector2 operator*(Type scalar) const
	{
		return Matrix21::operator*(scalar);
	}

	Type operator*(Vector2 b) const
	{
		return Vector<Type, 2>::operator*(b);
	}

	Type operator%(const Matrix21 &b) const
	{
		return (*this).cross(b);
	}

};

using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

} // namespace matrix
