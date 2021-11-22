#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

template class matrix::Slice<float, 2, 3, 4, 5>; // so that we get full coverage results


int main()
{
	float data[9] = {0, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	SquareMatrix<float, 3> A(data);

	// Test row slicing
	Matrix<float, 2, 3> B_rowslice(A.slice<2, 3>(1, 0));
	float data_check_rowslice[6] = {
		4, 5, 6,
		7, 8, 10
	};
	Matrix<float, 2, 3> B_check_rowslice(data_check_rowslice);
	TEST(isEqual(B_rowslice, B_check_rowslice));

	// Test column slicing
	Matrix<float, 3, 2> B_colslice(A.slice<3, 2>(0, 1));
	float data_check_colslice[6] = {
		2, 3,
		5, 6,
		8, 10
	};
	Matrix<float, 3, 2> B_check_colslice(data_check_colslice);
	TEST(isEqual(B_colslice, B_check_colslice));

	// Test slicing both
	Matrix<float, 2, 2> B_bothslice(A.slice<2, 2>(1, 1));
	float data_check_bothslice[4] = {
		5, 6,
		8, 10
	};
	Matrix<float, 2, 2> B_check_bothslice(data_check_bothslice);
	TEST(isEqual(B_bothslice, B_check_bothslice));

	//Test block writing
	float data_2[4] = {
		11, 12,
		13, 14
	};

	Matrix<float, 2, 2> C(data_2);
	A.slice<2, 2>(1, 1) = C;

	float data_2_check[9] = {
		0, 2, 3,
		4, 11, 12,
		7, 13, 14
	};
	Matrix<float, 3, 3> D(data_2_check);
	TEST(isEqual(A, D));

	//Test writing to slices
	Matrix<float, 3, 1> E;
	E(0, 0) = -1;
	E(1, 0) = 1;
	E(2, 0) = 3;

	Matrix<float, 2, 1> F;
	F(0, 0) = 9;
	F(1, 0) = 11;

	E.slice<2, 1>(0, 0) = F;

	float data_3_check[3] = {9, 11, 3};
	Matrix<float, 3, 1> G(data_3_check);
	TEST(isEqual(E, G));
	TEST(isEqual(E, Matrix<float, 3, 1>(E.slice<3, 1>(0, 0))));

	Matrix<float, 2, 1> H = E.slice<2, 1>(0, 0);
	TEST(isEqual(H, F));

	float data_4_check[5] = {3, 11, 9, 0, 0};
	{
		// assigning row slices to each other
		const Matrix<float, 3, 1> J(data_3_check);
		Matrix<float, 5, 1> K;
		K.row(2) = J.row(0);
		K.row(1) = J.row(1);
		K.row(0) = J.row(2);

		Matrix<float, 5, 1> K_check(data_4_check);
		TEST(isEqual(K, K_check));
	}
	{
		// assigning col slices to each other
		const Matrix<float, 1, 3> J(data_3_check);
		Matrix<float, 1, 5> K;
		K.col(2) = J.col(0);
		K.col(1) = J.col(1);
		K.col(0) = J.col(2);

		Matrix<float, 1, 5> K_check(data_4_check);
		TEST(isEqual(K, K_check));
	}

	// check that slice of a slice works for reading
	const Matrix<float, 3, 3> cm33(data);
	Matrix<float, 2, 1> topRight = cm33.slice<2, 3>(0, 0).slice<2, 1>(0, 2);
	float top_right_check[2] = {3, 6};
	TEST(isEqual(topRight, Matrix<float, 2, 1>(top_right_check)));

	// check that slice of a slice works for writing
	Matrix<float, 3, 3> m33(data);
	m33.slice<2, 3>(0, 0).slice<2, 1>(0, 2) = Matrix<float, 2, 1>();
	const float data_check[9] = {0, 2, 0,
				     4, 5, 0,
				     7, 8, 10
				    };
	TEST(isEqual(m33, Matrix<float, 3, 3>(data_check)));

	// longerThan
	Vector3f v5;
	v5(0) = 3;
	v5(1) = 4;
	v5(2) = 9;
	TEST(v5.xy().longerThan(4.99f));
	TEST(!v5.xy().longerThan(5.f));
	TEST(isEqualF(5.f, v5.xy().norm()));

	// min/max
	TEST(m33.row(1).max() == 5);
	TEST(m33.col(0).min() == 0);
	TEST((m33.slice<2, 2>(1, 1).max()) == 10);

	// assign scalar value to slice
	Matrix<float, 3, 1> L;
	L(0, 0) = -1;
	L(1, 0) = 1;
	L(2, 0) = 3;

	L.slice<2, 1>(0, 0) = 0.0f;

	float data_5_check[3] = {0, 0, 3};
	Matrix<float, 3, 1> M(data_5_check);
	TEST(isEqual(L, M));

	// return diagonal elements
	float data_6[9] = {0, 2, 3,
			   4, 5, 6,
			   7, 8, 10
			  };
	SquareMatrix<float, 3> N(data_6);

	Vector3f v6 = N.slice<3, 3>(0, 0).diag();
	Vector3f v6_check = {0, 5, 10};
	TEST(isEqual(v6, v6_check));
	Vector2f v7 = N.slice<2, 3>(1, 0).diag();
	Vector2f v7_check = {4, 8};
	TEST(isEqual(v7, v7_check));
	Vector2f v8 = N.slice<3, 2>(0, 1).diag();
	Vector2f v8_check = {2, 6};
	TEST(isEqual(v8, v8_check));
	Vector2f v9(N.slice<1, 2>(1, 1));
	Vector2f v9_check = {5, 6};
	TEST(isEqual(v9, v9_check));
	Vector3f v10(N.slice<1, 3>(1, 0));
	Vector3f v10_check = {4, 5, 6};
	TEST(isEqual(v10, v10_check));

	// Different assignment operators
	SquareMatrix3f O(data);
	float operand_data [4] = {2, 1, -3, -1};
	const SquareMatrix<float, 2> operand(operand_data);

	O.slice<2, 2>(1, 0) += operand;
	float O_check_data_1 [9] = {0, 2, 3,  6, 6, 6,  4, 7, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_1)));

	O = SquareMatrix3f(data);
	O.slice<2, 1>(1, 1) += operand.slice<2, 1>(0, 0);
	float O_check_data_2 [9] = {0, 2, 3,  4, 7, 6,  7, 5, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_2)));

	O = SquareMatrix3f(data);
	O.slice<3, 3>(0, 0) += -1;
	float O_check_data_3 [9] = {-1, 1, 2,  3, 4, 5,  6, 7, 9};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_3)));

	O = SquareMatrix3f(data);
	O.col(1) += Vector3f{1, -2, 3};
	float O_check_data_4 [9] = {0, 3, 3,  4, 3, 6,  7, 11, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_4)));

	O = SquareMatrix3f(data);
	O.slice<2, 2>(1, 0) -= operand;
	float O_check_data_5 [9] = {0, 2, 3,  2, 4, 6,  10, 9, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_5)));

	O = SquareMatrix3f(data);
	O.slice<2, 1>(1, 1) -= operand.slice<2, 1>(0, 0);
	float O_check_data_6 [9] = {0, 2, 3,  4, 3, 6,  7, 11, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_6)));

	O = SquareMatrix3f(data);
	O.slice<3, 3>(0, 0) -= -1;
	float O_check_data_7 [9] = {1, 3, 4,  5, 6, 7,  8, 9, 11};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_7)));

	O = SquareMatrix3f(data);
	O.col(1) -= Vector3f{1, -2, 3};
	float O_check_data_8 [9] = {0, 1, 3,  4, 7, 6,  7, 5, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_8)));

	O = SquareMatrix3f(data);
	O.slice<2, 1>(1, 1) *= 5.f;
	float O_check_data_9 [9] = {0, 2, 3,  4, 25, 6,  7, 40, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_9)));

	O = SquareMatrix3f(data);
	O.slice<2, 1>(1, 1) /= 2.f;
	float O_check_data_10 [9] = {0, 2, 3,  4, 2.5, 6,  7, 4, 10};
	TEST(isEqual(O, SquareMatrix3f(O_check_data_10)));

	// Different operations
	O = SquareMatrix3f(data);
	SquareMatrix<float, 2> res_11(O.slice<2, 2>(1, 1) * 2.f);
	float O_check_data_11 [4] = {10, 12, 16, 20};
	TEST(isEqual(res_11, SquareMatrix<float, 2>(O_check_data_11)));

	O = SquareMatrix3f(data);
	SquareMatrix<float, 2> res_12(O.slice<2, 2>(1, 1) / 2.f);
	float O_check_data_12 [4] = {2.5, 3, 4, 5};
	TEST(isEqual(res_12, SquareMatrix<float, 2>(O_check_data_12)));
	return 0;
}

