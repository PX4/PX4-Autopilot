/**
 * @file Matrix.hpp
 *
 * A quaternion class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once
#include <Vector.hpp>
#include <Dcm.hpp>
#include <Euler.hpp>

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Euler;

template<typename Type>
class Quaternion : public Vector<Type, 4>
{
public:
	virtual ~Quaternion() {};

	Quaternion() : Vector<Type, 4>()
	{
		// TODO
		Quaternion &q = *this;
		q(0) = 1;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
	}

	Quaternion(const Dcm<Type> & dcm) {
		// TODO
		Quaternion &q = *this;
		q(0) = 0;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
	}

	Quaternion(const Euler<Type> & e) {
		// TODO
		Quaternion &q = *this;
		q(0) = 0;
		q(1) = 0;
		q(2) = 0;
		q(3) = 0;
	}

	Quaternion(Type a, Type b, Type c, Type d) : Vector<Type, 4>()
	{
		// TODO
		Quaternion &q = *this;
		q(0) = a;
		q(1) = b;
		q(2) = c;
		q(3) = d;
	}

	Quaternion operator*(const Quaternion &q) const
	{
		// TODO
		const Quaternion &p = *this;
		Quaternion r;
		r(0) = 0;
		r(1) = 0;
		r(2) = 0;
		r(3) = 0;
		return r;
	}

	Matrix<Type, 4, 1> derivative() const {
		// TODO
		Matrix<Type, 4, 1> d;
		return d;
	}

};

typedef Quaternion<float> Quatf;

}; // namespace matrix
