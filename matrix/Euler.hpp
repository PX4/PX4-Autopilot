/**
 * @file Euler.hpp
 *
 * Euler angle tait-bryan body 3-2-1
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Vector.hpp>
#include <Dcm.hpp>
#include <Quaternion.hpp>

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

template<typename Type>
class Euler : public Vector<Type, 3>
{
public:
	virtual ~Euler() {};

	Euler() : Vector<Type, 3>()
	{
	}

	Euler(Type roll, Type pitch, Type yaw) : Vector<Type, 3>()
	{
		Euler &v(*this);
		v(0) = roll;
		v(1) = pitch;
		v(2) = yaw;
	}

	Euler(const Dcm<Type> & dcm) {
		// TODO
		Euler &e = *this;
		e(0) = 0;
		e(1) = 0;
		e(2) = 0;
	}

	Euler(const Quaternion<Type> & q) {
		// TODO
		Euler &e = *this;
		e(0) = 0;
		e(1) = 0;
		e(2) = 0;
	}

};

typedef Euler<float> Eulerf;

}; // namespace matrix
