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

    for(size_t i=0; i<3; i++) {
        for (size_t j = 0; j < 3; j++) {
            TEST(fabs(data[i*3 + j] - m2(i,j)) < FLT_EPSILON);
        }
    }

    Matrix3f m_nan;
    m_nan.setNaN();
    for(size_t i=0; i<3; i++) {
        for (size_t j = 0; j < 3; j++) {
            TEST(isnan(m_nan(i,j)));
        }
    }
    TEST(m_nan.isAllNan());

    float data2d[3][3] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    m2 = Matrix3f(data2d);
    for(size_t i=0; i<3; i++) {
        for (size_t j = 0; j < 3; j++) {
            TEST(fabs(data[i*3 + j] - m2(i,j)) < FLT_EPSILON);
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

    TEST(isEqual(-m4, m4*(-1)));

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
    const Vector<float, 1> & s_vect = s;
    TEST(fabs(s - 1) < FLT_EPSILON);
    TEST(fabs(s_vect(0) - 1.0f) < FLT_EPSILON);

    Matrix<float, 1, 1> m5 = s;
    TEST(fabs(m5(0,0) - s) < FLT_EPSILON);

    Matrix<float, 2, 2> m6;
    m6.setRow(0, Vector2f(1, 2));
    float m7_array[] = {1,2,0,0};
    Matrix<float, 2, 2> m7(m7_array);
    TEST(isEqual(m6, m7));
    m6.setCol(0, Vector2f(3, 4));
    float m8_array[] = {3,2,4,0};
    Matrix<float, 2, 2> m8(m8_array);
    TEST(isEqual(m6, m8));

    m7.setNaN();
    TEST(m7 != m8);

    // check write_string()
    float comma[6] = {
        1.f, 12345.678f,
        12345.67891f, 12345.67891f,
        1112345.67891f, 12345.111111111f
    };
    Matrix<float, 3, 2> Comma(comma);
    const size_t len = 10*2*3 + 2 + 1;
    char buffer[len];
    Comma.write_string(buffer, len);
    char output[] = "\t       1\t12345.678\n\t12345.679\t12345.679\n\t1112345.6\t12345.111\n";
    for (size_t i = 0; i < len; i++) {
        TEST(buffer[i] == output[i]);
        if (buffer[i] == '\0') {
            break;
        }
    }

    // check print()
    // write
    FILE *fp = fopen("testoutput.txt", "w+");
    TEST(fp != nullptr);
    Comma.print(fp);
    TEST(!fclose(fp));
    // read
    fp = fopen("testoutput.txt", "r");
    TEST(fp != nullptr);
    TEST(!fseek(fp, 0, SEEK_SET));
    for (size_t i = 0; i < len; i++) {
        char c = static_cast<char>(fgetc(fp));
        if (c == '\n') {
            break;
        }
        printf("%d %d %c\n", static_cast<int>(i), c, c);
        TEST(c == output[i]);
    }
    TEST(!fclose(fp));

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
