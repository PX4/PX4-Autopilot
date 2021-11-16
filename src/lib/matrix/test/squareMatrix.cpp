#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	float data[9] = {1, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	SquareMatrix<float, 3> A(data);
	Vector3<float> diag_check(1, 5, 10);

	TEST(isEqual(A.diag(), diag_check));
	TEST(A.trace() - 16 < FLT_EPSILON);

	float data_check[9] = {
		1.01158503f,  0.02190432f,  0.03238144f,
		0.04349195f,  1.05428524f,  0.06539627f,
		0.07576783f,  0.08708946f,  1.10894048f
	};

	float dt = 0.01f;
	SquareMatrix<float, 3> eA = expm(SquareMatrix<float, 3>(A * dt), 5);
	SquareMatrix<float, 3> eA_check(data_check);
	TEST((eA - eA_check).abs().max() < 1e-3f);

	SquareMatrix<float, 2> A_bottomright = A.slice<2, 2>(1, 1);
	SquareMatrix<float, 2> A_bottomright2;
	A_bottomright2 = A.slice<2, 2>(1, 1);

	float data_bottomright[4] = {5, 6,
				     8, 10
				    };
	SquareMatrix<float, 2> bottomright_check(data_bottomright);
	TEST(isEqual(A_bottomright, bottomright_check));
	TEST(isEqual(A_bottomright2, bottomright_check));

	// test diagonal functions
	float data_4x4[16] = {1, 2, 3, 4,
			      5, 6, 7, 8,
			      9, 10, 11, 12,
			      13, 14, 15, 16
			     };

	SquareMatrix<float, 4> B(data_4x4);
	B.uncorrelateCovariance<1>(1);
	float data_B_check[16] = {1, 0, 3, 4,
				  0, 6, 0, 0,
				  9, 0, 11, 12,
				  13, 0, 15, 16
				 };
	SquareMatrix<float, 4> B_check(data_B_check);
	TEST(isEqual(B, B_check))

	SquareMatrix<float, 4> C(data_4x4);
	C.uncorrelateCovariance<2>(1);
	float data_C_check[16] = {1, 0, 0, 4,
				  0, 6, 0, 0,
				  0, 0, 11, 0,
				  13, 0, 0, 16
				 };
	SquareMatrix<float, 4> C_check(data_C_check);
	TEST(isEqual(C, C_check))

	SquareMatrix<float, 4> D(data_4x4);
	D.uncorrelateCovarianceSetVariance<2>(0, Vector2f{20, 21});
	float data_D_check[16] = {20, 0, 0, 0,
				  0, 21, 0, 0,
				  0, 0, 11, 12,
				  0, 0, 15, 16
				 };
	SquareMatrix<float, 4> D_check(data_D_check);
	TEST(isEqual(D, D_check))

	SquareMatrix<float, 4> E(data_4x4);
	E.uncorrelateCovarianceSetVariance<3>(1, 33);
	float data_E_check[16] = {1, 0, 0, 0,
				  0, 33, 0, 0,
				  0, 0, 33, 0,
				  0, 0, 0, 33
				 };
	SquareMatrix<float, 4> E_check(data_E_check);
	TEST(isEqual(E, E_check))

	// test symmetric functions
	SquareMatrix<float, 4> F(data_4x4);
	F.makeBlockSymmetric<2>(1);
	float data_F_check[16] = {1, 2, 3, 4,
				  5, 6, 8.5, 8,
				  9, 8.5, 11, 12,
				  13, 14, 15, 16
				 };
	SquareMatrix<float, 4> F_check(data_F_check);
	TEST(isEqual(F, F_check))
	TEST(F.isBlockSymmetric<2>(1));
	TEST(!F.isRowColSymmetric<2>(1));

	SquareMatrix<float, 4> G(data_4x4);
	G.makeRowColSymmetric<2>(1);
	float data_G_check[16] = {1, 3.5, 6, 4,
				  3.5, 6, 8.5, 11,
				  6, 8.5, 11, 13.5,
				  13, 11, 13.5, 16
				 };
	SquareMatrix<float, 4> G_check(data_G_check);
	TEST(isEqual(G, G_check));
	TEST(G.isBlockSymmetric<2>(1));
	TEST(G.isRowColSymmetric<2>(1));

	SquareMatrix<float, 4> H(data_4x4);
	H.makeBlockSymmetric<1>(1);
	float data_H_check[16] = {1, 2, 3, 4,
				  5, 6, 7, 8,
				  9, 10, 11, 12,
				  13, 14, 15, 16
				 };
	SquareMatrix<float, 4> H_check(data_H_check);
	TEST(isEqual(H, H_check))
	TEST(H.isBlockSymmetric<1>(1));
	TEST(!H.isRowColSymmetric<1>(1));

	SquareMatrix<float, 4> J(data_4x4);
	J.makeRowColSymmetric<1>(1);
	float data_J_check[16] = {1, 3.5, 3, 4,
				  3.5, 6, 8.5, 11,
				  9, 8.5, 11, 12,
				  13, 11, 15, 16
				 };
	SquareMatrix<float, 4> J_check(data_J_check);
	TEST(isEqual(J, J_check));
	TEST(J.isBlockSymmetric<1>(1));
	TEST(J.isRowColSymmetric<1>(1));
	TEST(!J.isBlockSymmetric<3>(1));

	float data_K[16] = {1, 2, 3, 4,
			    2, 3, 4, 11,
			    3, 4, 11, 12,
			    4, 11, 15, 16
			   };
	SquareMatrix<float, 4> K(data_K);
	TEST(!K.isRowColSymmetric<1>(2));
	return 0;
}

