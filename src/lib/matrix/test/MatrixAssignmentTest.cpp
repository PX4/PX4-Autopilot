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

using namespace matrix;

TEST(MatrixAssignmentTest, Assignment)
{
	Matrix3f m;
	m.setZero();
	m.zero();
	m(0, 0) = 1;
	m(0, 1) = 2;
	m(0, 2) = 3;
	m(1, 0) = 4;
	m(1, 1) = 5;
	m(1, 2) = 6;
	m(2, 0) = 7;
	m(2, 1) = 8;
	m(2, 2) = 9;

	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f m2(data);

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			EXPECT_FLOAT_EQ(data[i * 3 + j], m2(i, j));
		}
	}

	Matrix3f m_nan;
	m_nan.setNaN();

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			EXPECT_TRUE(std::isnan(m_nan(i, j)));
		}
	}

	EXPECT_TRUE(m_nan.isAllNan());
	EXPECT_FALSE(m_nan.isAllFinite());

	float data2d[3][3] = {
		{1, 2, 3},
		{4, 5, 6},
		{7, 8, 9}
	};
	m2 = Matrix3f(data2d);

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			EXPECT_FLOAT_EQ(data[i * 3 + j], m2(i, j));
		}
	}

	EXPECT_FALSE(m2.isAllNan());
	EXPECT_TRUE(m2.isAllFinite());

	float data_times_2[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f m3(data_times_2);

	EXPECT_EQ(m, m2);
	EXPECT_NE(m, m3);

	m2 *= 2;
	EXPECT_EQ(m2, m3);

	m2 /= 2;
	m2 -= 1;
	float data_minus_1[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	EXPECT_EQ(Matrix3f(data_minus_1), m2);

	m2 += 1;
	EXPECT_EQ(Matrix3f(data), m2);

	m3 -= m2;

	EXPECT_EQ(m3, m2);

	// set rows and columns to value
	Matrix3f m2e(data2d);

	float data2e_check1[3][3] = {
		{1, 11, 3},
		{4, 11, 6},
		{7, 11, 9}
	};
	Matrix3f m2e_check1(data2e_check1);

	float data2e_check2[3][3] = {
		{1, 11, 3},
		{4, 11, 6},
		{0, 0, 0}
	};
	Matrix3f m2e_check2(data2e_check2);

	m2e.setCol(1, 11);
	EXPECT_EQ(m2e, m2e_check1);
	m2e.setRow(2, 0);
	EXPECT_EQ(m2e, m2e_check2);

	float data_row_02_swap[9] = {
		7, 8, 9,
		4, 5, 6,
		1, 2, 3,
	};

	float data_col_02_swap[9] = {
		3, 2, 1,
		6, 5, 4,
		9, 8, 7
	};

	Matrix3f m4(data);

	EXPECT_EQ(-m4, m4 * (-1));

	// col swap
	m4.swapCols(0, 2);
	EXPECT_EQ(m4, Matrix3f(data_col_02_swap));
	m4.swapCols(0, 2);

	// row swap
	m4.swapRows(0, 2);
	EXPECT_EQ(m4, Matrix3f(data_row_02_swap));
	m4.swapRows(0, 2);

	// swapping with same row should do nothing
	m4.swapRows(0, 0);
	m4.swapRows(1, 1);
	m4.swapRows(2, 2);
	EXPECT_EQ(m4, Matrix3f(data));

	// swapping with same col should do nothing
	m4.swapCols(0, 0);
	m4.swapCols(1, 1);
	m4.swapCols(2, 2);
	EXPECT_EQ(m4, Matrix3f(data));

	EXPECT_EQ(m4.min(), 1);
	EXPECT_EQ((-m4).min(), -9);

	Scalar<float> s = 1;
	const Vector<float, 1> &s_vect = s;
	EXPECT_EQ(s, 1.0f);
	EXPECT_EQ(s_vect(0), 1.0f);

	Matrix<float, 1, 1> m5 = s;
	EXPECT_EQ(s, m5(0, 0));

	Matrix<float, 2, 2> m6;
	m6.setRow(0, Vector2f(1, 2));
	float m7_array[] = {1, 2, 0, 0};
	Matrix<float, 2, 2> m7(m7_array);
	EXPECT_EQ(m6, m7);
	m6.setCol(0, Vector2f(3, 4));
	float m8_array[] = {3, 2, 4, 0};
	Matrix<float, 2, 2> m8(m8_array);
	EXPECT_EQ(m6, m8);

	m7.setNaN();
	EXPECT_NE(m7, m8);

	// min, max, constrain matrix values with scalar
	float data_m9[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	float lower_bound = 7;
	float upper_bound = 11;
	float data_m9_lower_bounded[9] = {7, 7, 7, 8, 10, 12, 14, 16, 18};
	float data_m9_upper_bounded[9] = {2, 4, 6, 8, 10, 11, 11, 11, 11};
	float data_m9_lower_constrained[9] = {7, 7, 7, 8, 10, 11, 11, 11, 11};
	Matrix3f m9(data_m9);
	Matrix3f m9_lower_bounded(data_m9_lower_bounded);
	Matrix3f m9_upper_bounded(data_m9_upper_bounded);
	Matrix3f m9_lower_upper_constrained(data_m9_lower_constrained);
	EXPECT_EQ(max(m9, lower_bound), m9_lower_bounded);
	EXPECT_EQ(max(lower_bound, m9), m9_lower_bounded);
	EXPECT_EQ(min(m9, upper_bound), m9_upper_bounded);
	EXPECT_EQ(min(upper_bound, m9), m9_upper_bounded);
	EXPECT_EQ(constrain(m9, lower_bound, upper_bound), m9_lower_upper_constrained);
	EXPECT_EQ(constrain(m9, 8.0f, 7.0f), m_nan);

	// min, max, constrain matrix values with matrix of same size
	float data_m10[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	float data_m10_lower_bound[9] = {5, 7, 4, 8, 19, 10, 20, 16, 18};
	float data_m10_lower_bounded_ref[9] = {5, 7, 6, 8, 19, 12, 20, 16, 18};
	float data_m10_upper_bound[9] = {6, 4, 8, 18, 20, 11, 30, 16, 18};
	float data_m10_upper_bounded_ref[9] = {2, 4, 6, 8, 10, 11, 14, 16, 18};
	float data_m10_constrained_ref[9] = {5, NAN, 6, 8, 19, 11, 20, 16, 18};
	Matrix3f m10(data_m10);
	Matrix3f m10_lower_bound(data_m10_lower_bound);
	Matrix3f m10_lower_bounded_ref(data_m10_lower_bounded_ref);
	Matrix3f m10_upper_bound(data_m10_upper_bound);
	Matrix3f m10_upper_bounded_ref(data_m10_upper_bounded_ref);
	Matrix3f m10_constrained_ref(data_m10_constrained_ref);
	EXPECT_EQ(max(m10, m10_lower_bound), m10_lower_bounded_ref);
	EXPECT_EQ(max(m10_lower_bound, m10), m10_lower_bounded_ref);
	EXPECT_EQ(min(m10, m10_upper_bound), m10_upper_bounded_ref);
	EXPECT_EQ(min(m10_upper_bound, m9), m10_upper_bounded_ref);
	EXPECT_EQ(constrain(m10, m10_lower_bound, m10_upper_bound), m10_constrained_ref);

	// min, max, constrain with NAN
	EXPECT_TRUE(isEqualF(matrix::typeFunction::min(5.f, NAN), 5.f));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::min(NAN, 5.f), 5.f));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::min(NAN, NAN), NAN));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::max(5.f, NAN), 5.f));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::max(NAN, 5.f), 5.f));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::max(NAN, NAN), NAN));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::constrain(NAN, 5.f, 6.f), NAN));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::constrain(1.f, 5.f, 4.f), NAN));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::constrain(6.f, NAN, 5.f), 5.f));
	EXPECT_TRUE(isEqualF(matrix::typeFunction::constrain(1.f, 5.f, NAN), 5.f));
	Vector2f v1{NAN, 5.0f};
	Vector2f v1_min = min(v1, 1.f);
	Matrix3f m11 = min(m10_constrained_ref, NAN);
	EXPECT_FLOAT_EQ(fmin(NAN, 1.f), float(v1_min(0)));
	EXPECT_EQ(m11, m10_constrained_ref);

	// check write_string()
	float comma[6] = {
		1.f, 12345.123f,
		12345.1228f, .1234567891011f,
		12345678910.123456789f, 1234567891011.123456789101112f
	};
	Matrix<float, 3, 2> Comma(comma);
	const size_t len = 15 * 2 * 3 + 2 + 1;
	char buffer[len];
	Comma.print(); // for debugging in case of failure
	Comma.write_string(buffer, len);
	printf("%s\n", buffer); // for debugging in case of failure
	char output[] = "\t       1\t12345.123\n\t12345.123\t0.12345679\n\t1.2345679e+10\t1.234568e+12\n";
	printf("%s\n", output); // for debugging in case of failure

	for (size_t i = 0; i < len; i++) {
		if (buffer[i] != output[i]) { // for debugging in case of failure
			printf("%d: \"%c\" != \"%c\"", int(i), buffer[i], output[i]); // LCOV_EXCL_LINE only print on failure
		}

		EXPECT_EQ(buffer[i], output[i]);

		if (buffer[i] == '\0') {
			break;
		}
	}

	char print_out[] = "  | 0      | 1      \n 0| 1.00000  1.2e+04\n 1| 1.2e+04  0.12346\n 2| 1.2e+10  1.2e+12\n";
	printf("%s\n", print_out); // for debugging in case of failure

	// check print()
	// Redirect stdout
	FILE *fp = freopen("testoutput.txt", "w", stdout);
	EXPECT_NE(fp, nullptr);

	// write
	Comma.print();
	EXPECT_FALSE(fclose(fp)); // FIXME: this doesn't work as expected, further printf are not redirected to the console

	// read
	fp = fopen("testoutput.txt", "r");
	EXPECT_NE(fp, nullptr);
	EXPECT_FALSE(fseek(fp, 0, SEEK_SET));

	for (size_t i = 0; i < len; i++) {
		char c = static_cast<char>(fgetc(fp));

		if (c == '\n') {
			break;
		}

		printf("%d %d %d\n", static_cast<int>(i), print_out[i], c);
		EXPECT_EQ(c, print_out[i]);
	}

	EXPECT_FALSE(fclose(fp));
}
