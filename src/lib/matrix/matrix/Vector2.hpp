/**
 * @file Vector2.hpp
 *
 * 2D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M>
class Vector;

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

	template<size_t P, size_t Q>
	Vector2(const Slice<Type, 2, 1, P, Q> &slice_in) : Vector<Type, 2>(slice_in)
	{
	}

	template<size_t P, size_t Q>
	Vector2(const Slice<Type, 1, 2, P, Q> &slice_in) : Vector<Type, 2>(slice_in)
	{
	}

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

	Type operator%(const Matrix21 &b) const
	{
		return (*this).cross(b);
	}

	/**
	 * @brief rotate vector
	 *
	 * @param[in] angle [rad]	The angle around the positive out of plane axis.
	 */
	void rotate(const Type angle)
	{
		Vector2 &v(*this);
		Vector2 rotated;
		Type sin_angle = Type(sin(angle));
		Type cos_angle = Type(cos(angle));
		rotated(0) = cos_angle * v(0) - sin_angle * v(1);
		rotated(1) = sin_angle * v(0) + cos_angle * v(1);
		v = rotated;
	}

	/**
	 * @brief Transform vector in new coordinate system
	 *
	 * @param[in] angle [rad]	The rotation of new coordinate system around the positive out of plane axis.
	 */
	void transform(const Type angle)
	{
		this->rotate(-angle);
	}

};


using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

} // namespace matrix
