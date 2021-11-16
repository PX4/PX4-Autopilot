#include "test_macros.hpp"
#include <matrix/integration.hpp>

using namespace matrix;

Vector<float, 6> f(float t, const Matrix<float, 6, 1> &  /*y*/, const Matrix<float, 3, 1> &  /*u*/);

Vector<float, 6> f(float t, const Matrix<float, 6, 1> &  /*y*/, const Matrix<float, 3, 1> &  /*u*/)
{
	float v = -sin(t);
	return v * ones<float, 6, 1>();
}

int main()
{
	Vector<float, 6> y = ones<float, 6, 1>();
	Vector<float, 3> u = ones<float, 3, 1>();
	float t0 = 0;
	float tf = 2;
	float h = 0.001f;
	integrate_rk4(f, y, u, t0, tf, h, y);
	float v = 1 + cos(tf) - cos(t0);
	TEST(isEqual(y, (ones<float, 6, 1>()*v)));
	return 0;
}

