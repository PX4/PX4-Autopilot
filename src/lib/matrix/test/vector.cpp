#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	// test data
	float data1[] = {1, 2, 3, 4, 5};
	float data2[] = {6, 7, 8, 9, 10};
	Vector<float, 5> v1(data1);
	Vector<float, 5> v2(data2);

	// copy constructor
	Vector<float, 5> v3(v2);
	TEST(isEqual(v2, v3));

	// norm, dot product
	TEST(isEqualF(v1.norm(), 7.416198487095663f));
	TEST(isEqualF(v1.norm_squared(), v1.norm() * v1.norm()));
	TEST(isEqualF(v1.norm(), v1.length()));
	TEST(isEqualF(v1.dot(v2), 130.0f));
	TEST(isEqualF(v1.dot(v2), v1 * v2));

	// unit, unit_zero, normalize
	TEST(isEqualF(v2.unit().norm(), 1.f));
	TEST(isEqualF(v2.unit_or_zero().norm(), 1.f));
	TEST(isEqualF(Vector<float, 5>().unit_or_zero().norm(), 0.f));
	v2.normalize();
	TEST(isEqualF(v2.norm(), 1.f));

	// sqrt
	float data1_sq[] = {1, 4, 9, 16, 25};
	Vector<float, 5> v4(data1_sq);
	TEST(isEqual(v1, v4.sqrt()));

	// longerThan
	Vector<float, 2> v5;
	v5(0) = 3;
	v5(1) = 4;
	TEST(v5.longerThan(4.99f));
	TEST(!v5.longerThan(5.f));

	return 0;
}

