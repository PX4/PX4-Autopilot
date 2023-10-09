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

#include <gtest/gtest.h>
#include <matrix/math.hpp>

using namespace matrix;

TEST(MatrixSquareTest, Square)
{
	float data[9] = {1, 2, 3,
			 4, 5, 6,
			 7, 8, 10
			};
	SquareMatrix<float, 3> A(data);
	Vector3<float> diag_check(1, 5, 10);

	EXPECT_EQ(A.diag(), diag_check);
	EXPECT_FLOAT_EQ(A.trace(), 16);
	EXPECT_FLOAT_EQ(A.trace<2>(1), 15);

	float data_check[9] = {
		1.01158503f,  0.02190432f,  0.03238144f,
		0.04349195f,  1.05428524f,  0.06539627f,
		0.07576783f,  0.08708946f,  1.10894048f
	};

	float dt = 0.01f;
	SquareMatrix<float, 3> eA = expm(SquareMatrix<float, 3>(A * dt), 5);
	SquareMatrix<float, 3> eA_check(data_check);
	EXPECT_TRUE(isEqual(eA, eA_check, 1e-3f));

	SquareMatrix<float, 2> A_bottomright = A.slice<2, 2>(1, 1);
	SquareMatrix<float, 2> A_bottomright2;
	A_bottomright2 = A.slice<2, 2>(1, 1);

	float data_bottomright[4] = {5, 6,
				     8, 10
				    };
	SquareMatrix<float, 2> bottomright_check(data_bottomright);
	EXPECT_EQ(A_bottomright, bottomright_check);
	EXPECT_EQ(A_bottomright2, bottomright_check);

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
	EXPECT_EQ(B, B_check);

	SquareMatrix<float, 4> C(data_4x4);
	C.uncorrelateCovariance<2>(1);
	float data_C_check[16] = {1, 0, 0, 4,
				  0, 6, 0, 0,
				  0, 0, 11, 0,
				  13, 0, 0, 16
				 };
	SquareMatrix<float, 4> C_check(data_C_check);
	EXPECT_EQ(C, C_check);

	SquareMatrix<float, 4> D(data_4x4);
	D.uncorrelateCovarianceSetVariance<2>(0, Vector2f{20, 21});
	float data_D_check[16] = {20, 0, 0, 0,
				  0, 21, 0, 0,
				  0, 0, 11, 12,
				  0, 0, 15, 16
				 };
	SquareMatrix<float, 4> D_check(data_D_check);
	EXPECT_EQ(D, D_check);

	SquareMatrix<float, 4> E(data_4x4);
	E.uncorrelateCovarianceSetVariance<3>(1, 33);
	float data_E_check[16] = {1, 0, 0, 0,
				  0, 33, 0, 0,
				  0, 0, 33, 0,
				  0, 0, 0, 33
				 };
	SquareMatrix<float, 4> E_check(data_E_check);
	EXPECT_EQ(E, E_check);

	SquareMatrix<float, 4> A_block(data_4x4);
	A_block.uncorrelateCovarianceBlock<2>(1);
	float data_A_block_check[16] = {1, 0, 0, 4,
					0, 6, 7, 0,
					0, 10, 11, 0,
					13, 0, 0, 16
				       };
	SquareMatrix<float, 4> A_block_check(data_A_block_check);
	EXPECT_EQ(A_block, A_block_check);

	// test symmetric functions
	SquareMatrix<float, 4> F(data_4x4);
	F.makeBlockSymmetric<2>(1);
	float data_F_check[16] = {1, 2, 3, 4,
				  5, 6, 8.5, 8,
				  9, 8.5, 11, 12,
				  13, 14, 15, 16
				 };
	SquareMatrix<float, 4> F_check(data_F_check);
	EXPECT_EQ(F, F_check);
	EXPECT_TRUE(F.isBlockSymmetric<2>(1));
	EXPECT_FALSE(F.isRowColSymmetric<2>(1));

	SquareMatrix<float, 4> G(data_4x4);
	G.makeRowColSymmetric<2>(1);
	float data_G_check[16] = {1, 3.5, 6, 4,
				  3.5, 6, 8.5, 11,
				  6, 8.5, 11, 13.5,
				  13, 11, 13.5, 16
				 };
	SquareMatrix<float, 4> G_check(data_G_check);
	EXPECT_EQ(G, G_check);
	EXPECT_TRUE(G.isBlockSymmetric<2>(1));
	EXPECT_TRUE(G.isRowColSymmetric<2>(1));

	SquareMatrix<float, 4> H(data_4x4);
	H.makeBlockSymmetric<1>(1);
	float data_H_check[16] = {1, 2, 3, 4,
				  5, 6, 7, 8,
				  9, 10, 11, 12,
				  13, 14, 15, 16
				 };
	SquareMatrix<float, 4> H_check(data_H_check);
	EXPECT_EQ(H, H_check);
	EXPECT_TRUE(H.isBlockSymmetric<1>(1));
	EXPECT_FALSE(H.isRowColSymmetric<1>(1));

	SquareMatrix<float, 4> J(data_4x4);
	J.makeRowColSymmetric<1>(1);
	float data_J_check[16] = {1, 3.5, 3, 4,
				  3.5, 6, 8.5, 11,
				  9, 8.5, 11, 12,
				  13, 11, 15, 16
				 };
	SquareMatrix<float, 4> J_check(data_J_check);
	EXPECT_EQ(J, J_check);
	EXPECT_TRUE(J.isBlockSymmetric<1>(1));
	EXPECT_TRUE(J.isRowColSymmetric<1>(1));
	EXPECT_FALSE(J.isBlockSymmetric<3>(1));

	float data_K[16] = {1, 2, 3, 4,
			    2, 3, 4, 11,
			    3, 4, 11, 12,
			    4, 11, 15, 16
			   };
	SquareMatrix<float, 4> K(data_K);
	EXPECT_FALSE(K.isRowColSymmetric<1>(2));

	float data_L[16] = {1, 0, 0, 0,
			    2, 3, 0, 0,
			    3, 4, 11, 0,
			    4, 11, 15, 16
			   };
	float data_L_check[16] = {1, 2, 3, 4,
				  2, 3, 4, 11,
				  3, 4, 11, 15,
				  4, 11, 15, 16
				 };
	SquareMatrix<float, 4> L(data_L);
	L.copyLowerToUpperTriangle();
	SquareMatrix<float, 4> L_check(data_L_check);
	EXPECT_EQ(L, L_check);

	float data_M[16] = {1, 2, 3, 4,
			    0, 3, 4, 11,
			    0, 0, 11, 15,
			    0, 0, 0, 16
			   };
	SquareMatrix<float, 4> M(data_M);
	M.copyUpperToLowerTriangle();
	EXPECT_EQ(M, L_check);
}
