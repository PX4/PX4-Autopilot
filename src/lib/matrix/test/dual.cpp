#include "test_macros.hpp"
#include <matrix/math.hpp>
#include <iostream>

using namespace matrix;

template <typename Scalar, size_t N>
bool isEqualAll(Dual<Scalar, N> a, Dual<Scalar, N> b)
{
	return isEqualF(a.value, b.value) && a.derivative == b.derivative;
}

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
	return positionMeasurement - (positionState +  bodyOrientation.conjugate(velocityStateBody) * dt);
}

int main()
{
	const Dual<float, 1> a(3, 0);
	const Dual<float, 1> b(6, 0);

	{
		TEST(isEqualF(a.value, 3.f));
		TEST(isEqualF(a.derivative(0), 1.f));
	}

	{
		// addition
		Dual<float, 1> c = a + b;
		TEST(isEqualF(c.value, 9.f));
		TEST(isEqualF(c.derivative(0), 2.f));

		Dual<float, 1> d = +a;
		TEST(isEqualAll(d, a));
		d += b;
		TEST(isEqualAll(d, c));

		Dual<float, 1> e = a;
		e += b.value;
		TEST(isEqualF(e.value, c.value));
		TEST(isEqual(e.derivative, a.derivative));

		Dual<float, 1> f = b.value + a;
		TEST(isEqualAll(f, e));
	}

	{
		// subtraction
		Dual<float, 1> c = b - a;
		TEST(isEqualF(c.value, 3.f));
		TEST(isEqualF(c.derivative(0), 0.f));

		Dual<float, 1> d = b;
		TEST(isEqualAll(d, b));
		d -= a;
		TEST(isEqualAll(d, c));

		Dual<float, 1> e = b;
		e -= a.value;
		TEST(isEqualF(e.value, c.value));
		TEST(isEqual(e.derivative, b.derivative));

		Dual<float, 1> f = a.value - b;
		TEST(isEqualAll(f, -e));
	}

	{
		// multiplication
		Dual<float, 1> c = a * b;
		TEST(isEqualF(c.value, 18.f));
		TEST(isEqualF(c.derivative(0), 9.f));

		Dual<float, 1> d = a;
		TEST(isEqualAll(d, a));
		d *= b;
		TEST(isEqualAll(d, c));

		Dual<float, 1> e = a;
		e *= b.value;
		TEST(isEqualF(e.value, c.value));
		TEST(isEqual(e.derivative, a.derivative * b.value));

		Dual<float, 1> f = b.value * a;
		TEST(isEqualAll(f, e));
	}

	{
		// division
		Dual<float, 1> c = b / a;
		TEST(isEqualF(c.value, 2.f));
		TEST(isEqualF(c.derivative(0), -1.f / 3.f));

		Dual<float, 1> d = b;
		TEST(isEqualAll(d, b));
		d /= a;
		TEST(isEqualAll(d, c));

		Dual<float, 1> e = b;
		e /= a.value;
		TEST(isEqualF(e.value, c.value));
		TEST(isEqual(e.derivative, b.derivative / a.value));

		Dual<float, 1> f = a.value / b;
		TEST(isEqualAll(f, 1.f / e));
	}

	{
		Dual<float, 1> blank;
		TEST(isEqualF(blank.value, 0.f));
		TEST(isEqualF(blank.derivative(0), 0.f));
	}

	{
		// sqrt
		TEST(isEqualF(sqrt(a).value, sqrt(a.value)));
		TEST(isEqualF(sqrt(a).derivative(0), 1.f / sqrt(12.f)));
	}

	{
		// abs
		TEST(isEqualAll(a, abs(-a)));
		TEST(!isEqualAll(-a, abs(a)));
		TEST(isEqualAll(-a, -abs(a)));
	}

	{
		// ceil
		Dual<float, 1> c(1.5, 0);
		TEST(isEqualF(ceil(c).value, ceil(c.value)));
		TEST(isEqualF(ceil(c).derivative(0), 0.f));
	}

	{
		// floor
		Dual<float, 1> c(1.5, 0);
		TEST(isEqualF(floor(c).value, floor(c.value)));
		TEST(isEqualF(floor(c).derivative(0), 0.f));
	}

	{
		// fmod
		TEST(isEqualF(fmod(a, 0.8f).value, fmod(a.value, 0.8f)));
		TEST(isEqual(fmod(a, 0.8f).derivative, a.derivative));
	}

	{
		// max/min
		TEST(isEqualAll(b, max(a, b)));
		TEST(isEqualAll(a, min(a, b)));
	}

	{
		// isnan
		TEST(!IsNan(a));
		Dual<float, 1> c(sqrt(-1.f), 0);
		TEST(IsNan(c));
	}

	{
		// isfinite/isinf
		TEST(IsFinite(a));
		TEST(!IsInf(a));
		Dual<float, 1> c(sqrt(-1.f), 0);
		TEST(!IsFinite(c));
		TEST(!IsInf(c));
		Dual<float, 1> d(INFINITY, 0);
		TEST(!IsFinite(d));
		TEST(IsInf(d));
	}

	{
		// sin/cos/tan
		TEST(isEqualF(sin(a).value, sin(a.value)));
		TEST(isEqualF(sin(a).derivative(0), cos(a.value))); // sin'(x) = cos(x)

		TEST(isEqualF(cos(a).value, cos(a.value)));
		TEST(isEqualF(cos(a).derivative(0), -sin(a.value))); // cos'(x) = -sin(x)

		TEST(isEqualF(tan(a).value, tan(a.value)));
		TEST(isEqualF(tan(a).derivative(0), 1.f + tan(a.value)*tan(a.value))); // tan'(x) = 1 + tan^2(x)
	}

	{
		// asin/acos/atan
		Dual<float, 1> c(0.3f, 0);
		TEST(isEqualF(asin(c).value, asin(c.value)));
		TEST(isEqualF(asin(c).derivative(0), 1.f / sqrt(1.f - 0.3f * 0.3f))); // asin'(x) = 1/sqrt(1-x^2)

		TEST(isEqualF(acos(c).value, acos(c.value)));
		TEST(isEqualF(acos(c).derivative(0), -1.f / sqrt(1.f - 0.3f * 0.3f))); // acos'(x) = -1/sqrt(1-x^2)

		TEST(isEqualF(atan(c).value, atan(c.value)));
		TEST(isEqualF(atan(c).derivative(0), 1.f / (1.f + 0.3f * 0.3f))); // tan'(x) = 1 + x^2
	}

	{
		// atan2
		TEST(isEqualF(atan2(a, b).value, atan2(a.value, b.value)));
		TEST(isEqualAll(atan2(a, Dual<float, 1>(b.value)), atan(a / b.value))); // atan2'(y, x) = atan'(y/x)
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

		TEST(isEqualF(dualResult.value, floatResult, 0.0f));
		TEST(isEqual(dualResult.derivative, numerical_derivative, 1e-2f));

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
		TEST(isEqual(direct_error, auto_error, 0.0f));
		TEST(isEqual(numerical_jacobian, auto_jacobian, 1e-3f));

	}
	return 0;
}
