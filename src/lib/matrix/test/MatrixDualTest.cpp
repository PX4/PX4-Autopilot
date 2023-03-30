/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <cmath>

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <iostream>

using namespace matrix;

template <typename T>
T testFunction(const Vector<T, 3> &point)
{
	// function is f(x,y,z) = x^2 + 2xy + 3y^2 + z
	return point(0) * point(0)
	       + 2.f * point(0) * point(1)
	       + 3.f * point(1) * point(1)
	       + point(2);
}

template <typename Scalar>
Vector<Scalar, 3> positionError(const Vector<Scalar, 3> &positionState,
				const Vector<Scalar, 3> &velocityStateBody,
				const Quaternion<Scalar> &bodyOrientation,
				const Vector<Scalar, 3> &positionMeasurement,
				Scalar dt
			       )
{
	return positionMeasurement - (positionState +  bodyOrientation.rotateVector(velocityStateBody) * dt);
}

TEST(MatrixDualTest, Dual)
{
	const Dual<float, 1> a(3, 0);
	const Dual<float, 1> b(6, 0);

	{
		EXPECT_FLOAT_EQ(a.value, 3.f);
		EXPECT_FLOAT_EQ(a.derivative(0), 1.f);
	}

	{
		// addition
		Dual<float, 1> c = a + b;
		EXPECT_FLOAT_EQ(c.value, 9.f);
		EXPECT_FLOAT_EQ(c.derivative(0), 2.f);

		Dual<float, 1> d = +a;
		EXPECT_EQ(d, a);
		d += b;
		EXPECT_EQ(d, c);

		Dual<float, 1> e = a;
		e += b.value;
		EXPECT_FLOAT_EQ(e.value, c.value);
		EXPECT_EQ(e.derivative, a.derivative);

		Dual<float, 1> f = b.value + a;
		EXPECT_EQ(f, e);
	}

	{
		// subtraction
		Dual<float, 1> c = b - a;
		EXPECT_FLOAT_EQ(c.value, 3.f);
		EXPECT_FLOAT_EQ(c.derivative(0), 0.f);

		Dual<float, 1> d = b;
		EXPECT_EQ(d, b);
		d -= a;
		EXPECT_EQ(d, c);

		Dual<float, 1> e = b;
		e -= a.value;
		EXPECT_FLOAT_EQ(e.value, c.value);
		EXPECT_EQ(e.derivative, b.derivative);

		Dual<float, 1> f = a.value - b;
		EXPECT_EQ(f, -e);
	}

	{
		// multiplication
		Dual<float, 1> c = a * b;
		EXPECT_FLOAT_EQ(c.value, 18.f);
		EXPECT_FLOAT_EQ(c.derivative(0), 9.f);

		Dual<float, 1> d = a;
		EXPECT_EQ(d, a);
		d *= b;
		EXPECT_EQ(d, c);

		Dual<float, 1> e = a;
		e *= b.value;
		EXPECT_FLOAT_EQ(e.value, c.value);
		EXPECT_EQ(e.derivative, a.derivative * b.value);

		Dual<float, 1> f = b.value * a;
		EXPECT_EQ(f, e);
	}

	{
		// division
		Dual<float, 1> c = b / a;
		EXPECT_FLOAT_EQ(c.value, 2.f);
		EXPECT_FLOAT_EQ(c.derivative(0), -1.f / 3.f);

		Dual<float, 1> d = b;
		EXPECT_EQ(d, b);
		d /= a;
		EXPECT_EQ(d, c);

		Dual<float, 1> e = b;
		e /= a.value;
		EXPECT_FLOAT_EQ(e.value, c.value);
		EXPECT_EQ(e.derivative, b.derivative / a.value);

		Dual<float, 1> f = a.value / b;
		EXPECT_EQ(f, 1.f / e);
	}

	{
		Dual<float, 1> blank;
		EXPECT_FLOAT_EQ(blank.value, 0.f);
		EXPECT_FLOAT_EQ(blank.derivative(0), 0.f);
	}

	{
		// sqrt
		EXPECT_FLOAT_EQ(sqrt(a).value, std::sqrt(a.value));
		EXPECT_FLOAT_EQ(sqrt(a).derivative(0), 1.f / sqrt(12.f));
	}

	{
		// abs
		EXPECT_EQ(a, abs(-a));
		EXPECT_NE(-a, abs(a));
		EXPECT_EQ(-a, -abs(a));
	}

	{
		// ceil
		Dual<float, 1> c(1.5, 0);
		EXPECT_FLOAT_EQ(ceil(c).value, ceil(c.value));
		EXPECT_FLOAT_EQ(ceil(c).derivative(0), 0.f);
	}

	{
		// floor
		Dual<float, 1> c(1.5, 0);
		EXPECT_FLOAT_EQ(floor(c).value, floor(c.value));
		EXPECT_FLOAT_EQ(floor(c).derivative(0), 0.f);
	}

	{
		// fmod
		EXPECT_FLOAT_EQ(fmod(a, 0.8f).value, fmod(a.value, 0.8f));
		EXPECT_EQ(fmod(a, 0.8f).derivative, a.derivative);
	}

	{
		// max/min
		EXPECT_EQ(b, max(a, b));
		EXPECT_EQ(a, min(a, b));
	}

	{
		// isnan
		EXPECT_FALSE(IsNan(a));
		Dual<float, 1> c(std::sqrt(-1.f), 0);
		EXPECT_TRUE(IsNan(c));
	}

	{
		// isfinite/isinf
		EXPECT_TRUE(IsFinite(a));
		EXPECT_FALSE(IsInf(a));
		Dual<float, 1> c(std::sqrt(-1.f), 0);
		EXPECT_FALSE(IsFinite(c));
		EXPECT_FALSE(IsInf(c));
		Dual<float, 1> d(INFINITY, 0);
		EXPECT_FALSE(IsFinite(d));
		EXPECT_TRUE(IsInf(d));
	}

	{
		// sin/cos/tan
		EXPECT_FLOAT_EQ(sin(a).value, sin(a.value));
		EXPECT_FLOAT_EQ(sin(a).derivative(0), std::cos(a.value)); // sin'(x) = cos(x)

		EXPECT_FLOAT_EQ(cos(a).value, cos(a.value));
		EXPECT_FLOAT_EQ(cos(a).derivative(0), -std::sin(a.value)); // cos'(x) = -sin(x)

		EXPECT_FLOAT_EQ(tan(a).value, tan(a.value));
		EXPECT_FLOAT_EQ(tan(a).derivative(0), 1.f + std::tan(a.value)*std::tan(a.value)); // tan'(x) = 1 + tan^2(x)
	}

	{
		// asin/acos/atan
		Dual<float, 1> c(0.3f, 0);
		EXPECT_FLOAT_EQ(asin(c).value, std::asin(c.value));
		EXPECT_FLOAT_EQ(asin(c).derivative(0), 1.f / std::sqrt(1.f - 0.3f * 0.3f)); // asin'(x) = 1/sqrt(1-x^2)

		EXPECT_FLOAT_EQ(acos(c).value, std::acos(c.value));
		EXPECT_FLOAT_EQ(acos(c).derivative(0), -1.f / std::sqrt(1.f - 0.3f * 0.3f)); // acos'(x) = -1/sqrt(1-x^2)

		EXPECT_FLOAT_EQ(atan(c).value, std::atan(c.value));
		EXPECT_FLOAT_EQ(atan(c).derivative(0), 1.f / (1.f + 0.3f * 0.3f)); // tan'(x) = 1 + x^2
	}

	{
		// atan2
		EXPECT_FLOAT_EQ(atan2(a, b).value, atan2(a.value, b.value));
		EXPECT_EQ(atan2(a, Dual<float, 1>(b.value)), atan(a / b.value)); // atan2'(y, x) = atan'(y/x)
	}

	{
		// partial derivatives
		// function is f(x,y,z) = x^2 + 2xy + 3y^2 + z, we need with respect to d/dx and d/dy at the point (0.5, -0.8, 2)

		using D = Dual<float, 2>;

		// set our starting point, requesting partial derivatives of x and y in column 0 and 1
		Vector3<D> dualPoint(D(0.5f, 0), D(-0.8f, 1), D(2.f));

		Dual<float, 2> dualResult = testFunction(dualPoint);

		// compare to a numerical derivative:
		Vector<float, 3> floatPoint = collectReals(dualPoint);
		float floatResult = testFunction(floatPoint);

		float h = 0.0001f;
		Vector<float, 3> floatPoint_plusDX = floatPoint;
		floatPoint_plusDX(0) += h;
		float floatResult_plusDX = testFunction(floatPoint_plusDX);

		Vector<float, 3> floatPoint_plusDY = floatPoint;
		floatPoint_plusDY(1) += h;
		float floatResult_plusDY = testFunction(floatPoint_plusDY);

		Vector2f numerical_derivative((floatResult_plusDX - floatResult) / h,
					      (floatResult_plusDY - floatResult) / h);

		EXPECT_EQ(dualResult.value, floatResult);
		EXPECT_TRUE(isEqual(dualResult.derivative, numerical_derivative, 1e-2f));
	}

	{
		// jacobian
		// get residual of x/y/z with partial derivatives of rotation

		Vector3f direct_error;
		Matrix<float, 3, 4> numerical_jacobian;
		{
			Vector3f positionState(5, 6, 7);
			Vector3f velocityState(-1, 0, 1);
			Quaternionf velocityOrientation(0.2f, -0.1f, 0, 1);
			Vector3f positionMeasurement(4.5f, 6.2f, 7.9f);
			float dt = 0.1f;

			direct_error = positionError(positionState,
						     velocityState,
						     velocityOrientation,
						     positionMeasurement,
						     dt);
			float h = 0.001f;

			for (size_t i = 0; i < 4; i++) {
				Quaternion<float> h4 = velocityOrientation;
				h4(i) += h;
				numerical_jacobian.col(i) = (positionError(positionState,
							     velocityState,
							     h4,
							     positionMeasurement,
							     dt)
							     - direct_error) / h;
			}
		}
		Vector3f auto_error;
		Matrix<float, 3, 4> auto_jacobian;
		{
			using D4 = Dual<float, 4>;
			using Vector3d4 = Vector3<D4>;
			Vector3d4 positionState(D4(5), D4(6), D4(7));
			Vector3d4 velocityState(D4(-1), D4(0), D4(1));

			// request partial derivatives of velocity orientation
			// by setting these variables' derivatives in corresponding columns [0...3]
			Quaternion<D4> velocityOrientation(D4(0.2f, 0), D4(-0.1f, 1), D4(0, 2), D4(1, 3));

			Vector3d4 positionMeasurement(D4(4.5f), D4(6.2f), D4(7.9f));
			D4 dt(0.1f);


			Vector3d4 error = positionError(positionState,
							velocityState,
							velocityOrientation,
							positionMeasurement,
							dt);
			auto_error = collectReals(error);
			auto_jacobian = collectDerivatives(error);
		}
		EXPECT_EQ(direct_error, auto_error);
		EXPECT_TRUE(isEqual(numerical_jacobian, auto_jacobian, 1e-3f));
	}
}
