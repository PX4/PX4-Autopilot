#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	float data[9] = {1, 0, 0, 0, 1, 0, 1, 0, 1};
	Matrix3f A(data);

	float data_check[9] = {1, 0, 0, 0, 1, 0, -1, 0, 1};
	Matrix3f A_I(data_check);
	Matrix3f I;
	I.setIdentity();
	Matrix3f R = A * A_I;
	TEST(isEqual(R, I));

	Matrix3f R2 = A;
	R2 *= A_I;
	TEST(isEqual(R2, I));

	TEST(R2 == I);
	TEST(A != A_I);
	Matrix3f A2 = eye<float, 3>() * 2;
	Matrix3f B = A2.emult(A2);
	Matrix3f B_check = eye<float, 3>() * 4;
	Matrix3f C_check = eye<float, 3>() * 2;
	TEST(isEqual(B, B_check));
	Matrix3f C = B_check.edivide(C_check);

	float off_diagonal_nan[9] = {2, NAN, NAN, NAN, 2, NAN, NAN, NAN, 2};
	// off diagonal are NANs because division by 0
	TEST(C == Matrix3f(off_diagonal_nan));

	// Test non-square matrix
	float data_43[12] = {1, 3, 2,
			     2, 2, 1,
			     5, 2, 1,
			     2, 3, 4
			    };
	float data_32[6] = {2, 3,
			    1, 7,
			    5, 4
			   };

	Matrix<float, 4, 3> m43(data_43);
	Matrix<float, 3, 2> m32(data_32);

	Matrix<float, 4, 2> m42 = m43 * m32;

	float data_42[8] = {15, 32,
			    11, 24,
			    17, 33,
			    27, 43
			   };
	Matrix<float, 4, 2> m42_check(data_42);
	TEST(isEqual(m42, m42_check))

	float data_42_plus2[8] = {17, 34,
				  13, 26,
				  19, 35,
				  29, 45
				 };
	Matrix<float, 4, 2> m42_plus2_check(data_42_plus2);
	Matrix<float, 4, 2> m42_plus2 = m42 - (-2);
	TEST(isEqual(m42_plus2, m42_plus2_check));

	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
