/**
 * @file AxisAngle.hpp
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Euler;

template <typename Type>
class AxisAngle;

/**
 * AxisAngle class
 *
 * The rotation between two coordinate frames is
 * described by this class.
 */
template<typename Type>
class AxisAngle : public Vector<Type, 3>
{
public:
	using Matrix31 = Matrix<Type, 3, 1>;

	/**
	 * Constructor from array
	 *
	 * @param data_ array
	 */
	explicit AxisAngle(const Type data_[3]) :
		Vector<Type, 3>(data_)
	{
	}

	/**
	 * Standard constructor
	 */
	AxisAngle() = default;

	/**
	 * Constructor from Matrix31
	 *
	 * @param other Matrix31 to copy
	 */
	AxisAngle(const Matrix31 &other) :
		Vector<Type, 3>(other)
	{
	}

	/**
	 * Constructor from quaternion
	 *
	 * This sets the instance from a quaternion representing coordinate transformation from
	 * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
	 * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
	 *
	 * @param q quaternion
	 */
	AxisAngle(const Quaternion<Type> &q)
	{
		AxisAngle &v = *this;
		Type mag = q.imag().norm();

		if (fabs(mag) >= Type(1e-10)) {
			v = q.imag() * Type(Type(2) * atan2(mag, q(0)) / mag);

		} else {
			v = q.imag() * Type(Type(2) * Type(sign(q(0))));
		}
	}

	/**
	 * Constructor from dcm
	 *
	 * Instance is initialized from a dcm representing coordinate transformation
	 * from frame 2 to frame 1.
	 *
	 * @param dcm dcm to set quaternion to
	 */
	AxisAngle(const Dcm<Type> &dcm)
	{
		AxisAngle &v = *this;
		v = AxisAngle<Type>(Quaternion<Type>(dcm));
	}

	/**
	 * Constructor from euler angles
	 *
	 * This sets the instance to a quaternion representing coordinate transformation from
	 * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
	 * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
	 *
	 * @param euler euler angle instance
	 */
	AxisAngle(const Euler<Type> &euler)
	{
		AxisAngle &v = *this;
		v = AxisAngle<Type>(Quaternion<Type>(euler));
	}

	/**
	 * Constructor from 3 axis angle values (unit vector * angle)
	 *
	 * @param x r_x*angle
	 * @param y r_y*angle
	 * @param z r_z*angle
	 */
	AxisAngle(Type x, Type y, Type z)
	{
		AxisAngle &v = *this;
		v(0) = x;
		v(1) = y;
		v(2) = z;
	}

	/**
	 * Constructor from axis and angle
	 *
	 * @param axis An axis of rotation, normalized if not unit length
	 * @param angle The amount to rotate
	 */
	AxisAngle(const Matrix31 &axis_, Type angle_)
	{
		AxisAngle &v = *this;
		// make sure axis is a unit vector
		Vector<Type, 3> a = axis_;
		a = a.unit();
		v(0) = a(0) * angle_;
		v(1) = a(1) * angle_;
		v(2) = a(2) * angle_;
	}


	Vector<Type, 3> axis()
	{
		if (Vector<Type, 3>::norm() > 0) {
			return Vector<Type, 3>::unit();

		} else {
			return Vector3<Type>(1, 0, 0);
		}
	}

	Type angle()
	{
		return Vector<Type, 3>::norm();
	}
};

using AxisAnglef = AxisAngle<float>;
using AxisAngled = AxisAngle<double>;

} // namespace matrix
