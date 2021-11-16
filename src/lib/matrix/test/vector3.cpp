#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	Vector3f a(1, 0, 0);
	Vector3f b(0, 1, 0);
	Vector3f c = a.cross(b);
	TEST(isEqual(c, Vector3f(0, 0, 1)));
	c = a % b;
	TEST(isEqual(c, Vector3f(0, 0, 1)));
	Matrix<float, 3, 1> d(c);
	Vector3f e(d);
	TEST(isEqual(e, d));
	float data[] = {4, 5, 6};
	Vector3f f(data);
	TEST(isEqual(f, Vector3f(4, 5, 6)));

	TEST(isEqual(a + b, Vector3f(1, 1, 0)));
	TEST(isEqual(a - b, Vector3f(1, -1, 0)));
	TEST(isEqualF(a * b, 0.0f));
	TEST(isEqual(-a, Vector3f(-1, 0, 0)));
	TEST(isEqual(a.unit(), a));
	TEST(isEqual(a.unit(), a.normalized()));
	TEST(isEqual(a * 2.0, Vector3f(2, 0, 0)));

	Vector2f g2(1, 3);
	Vector3f g3(7, 11, 17);
	g3.xy() = g2;
	TEST(isEqual(g3, Vector3f(1, 3, 17)));

	const Vector3f g4(g3);
	Vector2f g5 = g4.xy();
	TEST(isEqual(g5, g2));
	TEST(isEqual(g2, Vector2f(g4.xy())));

	Vector3f h;
	TEST(isEqual(h, Vector3f(0, 0, 0)));

	Vector<float, 4> j;
	j(0) = 1;
	j(1) = 2;
	j(2) = 3;
	j(3) = 4;

	Vector3f k = j.slice<3, 1>(0, 0);
	Vector3f k_test(1, 2, 3);
	TEST(isEqual(k, k_test));

	Vector3f m1(1, 2, 3);
	Vector3f m2(3.1f, 4.1f, 5.1f);
	TEST(isEqual(m2, m1 + 2.1f));
	TEST(isEqual(m2 - 2.1f, m1));

	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
