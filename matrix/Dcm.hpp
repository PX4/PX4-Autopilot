/**
 * @file Dcm.hpp
 *
 * A direction cosine matrix class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <Matrix.hpp>
#include <Quaternion.hpp>
#include <Euler.hpp>

namespace matrix
{

template<typename Type>
class Quaternion;

template<typename Type>
class Dcm : public Matrix<Type, 3, 3>
{
public:
	virtual ~Dcm() {};

	typedef Matrix<Type, 3, 1> Vector3;

	Dcm() : Matrix<Type, 3, 3>()
	{
	}

	Dcm(const Quaternion<Type> & q) {
		// TODO
		Dcm &c = *this;
		c(0, 0) = 0;
		c(0, 1) = 0;
		c(0, 2) = 0;
		c(1, 0) = 0;
		c(1, 1) = 0;
		c(1, 2) = 0;
		c(2, 0) = 0;
		c(2, 1) = 0;
		c(2, 2) = 0;
	}

	Dcm(const Euler<Type> & e) {
		// TODO
		Dcm &c = *this;
		c(0, 0) = 0;
		c(0, 1) = 0;
		c(0, 2) = 0;
		c(1, 0) = 0;
		c(1, 1) = 0;
		c(1, 2) = 0;
		c(2, 0) = 0;
		c(2, 1) = 0;
		c(2, 2) = 0;
	}
};

typedef Dcm<float> Dcmf;

}; // namespace matrix
