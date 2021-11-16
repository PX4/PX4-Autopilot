/**
 * @file Scalar.hpp
 *
 * Defines conversion of matrix to scalar.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type>
class Scalar
{
public:
	Scalar() = delete;

	Scalar(const Matrix<Type, 1, 1> &other) :
		_value{other(0, 0)}
	{
	}

	Scalar(Type other) : _value(other)
	{
	}

	operator const Type &()
	{
		return _value;
	}

	operator Matrix<Type, 1, 1>() const
	{
		Matrix<Type, 1, 1> m;
		m(0, 0) = _value;
		return m;
	}

	operator Vector<Type, 1>() const
	{
		Vector<Type, 1> m;
		m(0) = _value;
		return m;
	}

private:
	const Type _value;

};

using Scalarf = Scalar<float>;
using Scalard = Scalar<double>;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
