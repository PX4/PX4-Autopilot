#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

template class matrix::Matrix<float, 3, 2>;

int main()
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
			TEST(fabs(data[i * 3 + j] - m2(i, j)) < FLT_EPSILON);
		}
	}

	Matrix3f m_nan;
	m_nan.setNaN();

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			TEST(isnan(m_nan(i, j)));
		}
	}

	TEST(m_nan.isAllNan());

	float data2d[3][3] = {
		{1, 2, 3},
		{4, 5, 6},
		{7, 8, 9}
	};
	m2 = Matrix3f(data2d);

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			TEST(fabs(data[i * 3 + j] - m2(i, j)) < FLT_EPSILON);
		}
	}

	TEST(!m2.isAllNan());

	float data_times_2[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f m3(data_times_2);

	TEST(isEqual(m, m2));
	TEST(!(isEqual(m, m3)));

	m2 *= 2;
	TEST(isEqual(m2, m3));

	m2 /= 2;
	m2 -= 1;
	float data_minus_1[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	TEST(isEqual(Matrix3f(data_minus_1), m2));

	m2 += 1;
	TEST(isEqual(Matrix3f(data), m2));

	m3 -= m2;

	TEST(isEqual(m3, m2));

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
	TEST(isEqual(m2e, m2e_check1));
	m2e.setRow(2, 0);
	TEST(isEqual(m2e, m2e_check2));

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

	TEST(isEqual(-m4, m4 * (-1)));

	// col swap
	m4.swapCols(0, 2);
	TEST(isEqual(m4, Matrix3f(data_col_02_swap)));
	m4.swapCols(0, 2);

	// row swap
	m4.swapRows(0, 2);
	TEST(isEqual(m4, Matrix3f(data_row_02_swap)));
	m4.swapRows(0, 2);

	// swapping with same row should do nothing
	m4.swapRows(0, 0);
	m4.swapRows(1, 1);
	m4.swapRows(2, 2);
	TEST(isEqual(m4, Matrix3f(data)));

	// swapping with same col should do nothing
	m4.swapCols(0, 0);
	m4.swapCols(1, 1);
	m4.swapCols(2, 2);
	TEST(isEqual(m4, Matrix3f(data)));

	TEST(fabs(m4.min() - 1) < FLT_EPSILON);
	TEST(fabs((-m4).min() + 9) < FLT_EPSILON);

	Scalar<float> s = 1;
	const Vector<float, 1> &s_vect = s;
	TEST(fabs(s - 1) < FLT_EPSILON);
	TEST(fabs(s_vect(0) - 1.0f) < FLT_EPSILON);

	Matrix<float, 1, 1> m5 = s;
	TEST(fabs(m5(0, 0) - s) < FLT_EPSILON);

	Matrix<float, 2, 2> m6;
	m6.setRow(0, Vector2f(1, 2));
	float m7_array[] = {1, 2, 0, 0};
	Matrix<float, 2, 2> m7(m7_array);
	TEST(isEqual(m6, m7));
	m6.setCol(0, Vector2f(3, 4));
	float m8_array[] = {3, 2, 4, 0};
	Matrix<float, 2, 2> m8(m8_array);
	TEST(isEqual(m6, m8));

	m7.setNaN();
	TEST(m7 != m8);

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
	TEST(isEqual(max(m9, lower_bound), m9_lower_bounded));
	TEST(isEqual(max(lower_bound, m9), m9_lower_bounded));
	TEST(isEqual(min(m9, upper_bound), m9_upper_bounded));
	TEST(isEqual(min(upper_bound, m9), m9_upper_bounded));
	TEST(isEqual(constrain(m9, lower_bound, upper_bound), m9_lower_upper_constrained));
	TEST(isEqual(constrain(m9, 8.0f, 7.0f), m_nan));

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
	TEST(isEqual(max(m10, m10_lower_bound), m10_lower_bounded_ref));
	TEST(isEqual(max(m10_lower_bound, m10), m10_lower_bounded_ref));
	TEST(isEqual(min(m10, m10_upper_bound), m10_upper_bounded_ref));
	TEST(isEqual(min(m10_upper_bound, m9), m10_upper_bounded_ref));
	TEST(isEqual(constrain(m10, m10_lower_bound, m10_upper_bound), m10_constrained_ref));

	// min, max, constrain with NAN
	TEST(isEqualF(matrix::typeFunction::min(5.f, NAN), 5.f));
	TEST(isEqualF(matrix::typeFunction::min(NAN, 5.f), 5.f));
	TEST(isEqualF(matrix::typeFunction::min(NAN, NAN), NAN));
	TEST(isEqualF(matrix::typeFunction::max(5.f, NAN), 5.f));
	TEST(isEqualF(matrix::typeFunction::max(NAN, 5.f), 5.f));
	TEST(isEqualF(matrix::typeFunction::max(NAN, NAN), NAN));
	TEST(isEqualF(matrix::typeFunction::constrain(NAN, 5.f, 6.f), NAN));
	TEST(isEqualF(matrix::typeFunction::constrain(1.f, 5.f, 4.f), NAN));
	TEST(isEqualF(matrix::typeFunction::constrain(6.f, NAN, 5.f), 5.f));
	TEST(isEqualF(matrix::typeFunction::constrain(1.f, 5.f, NAN), 5.f));
	Vector2f v1{NAN, 5.0f};
	Vector2f v1_min = min(v1, 1.f);
	Matrix3f m11 = min(m10_constrained_ref, NAN);
	TEST(isEqualF(fmin(NAN, 1.f), float(v1_min(0))));
	TEST(isEqual(m11, m10_constrained_ref));

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

		TEST(buffer[i] == output[i]);

		if (buffer[i] == '\0') {
			break;
		}
	}

	// check print()
	// Redirect stdout
	TEST(freopen("testoutput.txt", "w", stdout) != NULL);
	// write
	Comma.print();
	fclose(stdout);
	// read
	FILE *fp = fopen("testoutput.txt", "r");
	TEST(fp != nullptr);
	TEST(!fseek(fp, 0, SEEK_SET));

	for (size_t i = 0; i < len; i++) {
		char c = static_cast<char>(fgetc(fp));

		if (c == '\n') {
			break;
		}

		printf("%d %d %d\n", static_cast<int>(i), output[i], c);
		TEST(c == output[i]);
	}

	TEST(!fclose(fp));

	return 0;
}

