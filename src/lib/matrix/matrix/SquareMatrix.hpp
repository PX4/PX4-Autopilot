/**
 * @file SquareMatrix.hpp
 *
 * A square matrix
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template <typename Type, size_t M>
class Vector;

template <typename Type, size_t P, size_t Q, size_t M, size_t N>
class Slice;

template <typename Type, size_t  M>
class SquareMatrix : public Matrix<Type, M, M>
{
public:
    SquareMatrix() = default;

    explicit SquareMatrix(const Type data_[M][M]) :
        Matrix<Type, M, M>(data_)
    {
    }

    explicit SquareMatrix(const Type data_[M*M]) :
        Matrix<Type, M, M>(data_)
    {
    }

    SquareMatrix(const Matrix<Type, M, M> &other) :
        Matrix<Type, M, M>(other)
    {
    }

    template<size_t P, size_t Q>
    SquareMatrix(const Slice<Type, M, M, P, Q>& in_slice) : Matrix<Type, M, M>(in_slice)
    {
    }

    SquareMatrix<Type, M>& operator=(const Matrix<Type, M, M>& other)
    {
        Matrix<Type, M, M>::operator=(other);
        return *this;
    }

    template <size_t P, size_t Q>
    SquareMatrix<Type, M> & operator=(const Slice<Type, M, M, P, Q>& in_slice)
    {
        Matrix<Type, M, M>::operator=(in_slice);
        return *this;
    }

    template<size_t P, size_t Q>
    const Slice<Type, P, Q, M, M> slice(size_t x0, size_t y0) const
    {
        return Slice<Type, P, Q, M, M>(x0, y0, this);
    }

    template<size_t P, size_t Q>
    Slice<Type, P, Q, M, M> slice(size_t x0, size_t y0)
    {
        return Slice<Type, P, Q, M, M>(x0, y0, this);
    }

    // inverse alias
    inline SquareMatrix<Type, M> I() const
    {
        SquareMatrix<Type, M> i;
        if (inv(*this, i)) {
            return i;
        } else {
            i.setZero();
            return i;
        }
    }

    // inverse alias
    inline bool I(SquareMatrix<Type, M> &i) const
    {
        return inv(*this, i);
    }


    Vector<Type, M> diag() const
    {
        Vector<Type, M> res;
        const SquareMatrix<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res(i) = self(i, i);
        }
        return res;
    }

    // get matrix upper right triangle in a row-major vector format
    Vector<Type, M * (M + 1) / 2> upper_right_triangle() const
    {
        Vector<Type, M * (M + 1) / 2> res;
        const SquareMatrix<Type, M> &self = *this;

        unsigned idx = 0;
        for (size_t x = 0; x < M; x++) {
            for (size_t y = x; y < M; y++) {
                res(idx) = self(x, y);
                ++idx;
            }
        }

        return res;
    }

    Type trace() const
    {
        Type res = 0;
        const SquareMatrix<Type, M> &self = *this;

        for (size_t i = 0; i < M; i++) {
            res += self(i, i);
        }
        return res;
    }

    // zero all offdiagonal elements and keep corresponding diagonal elements
    template <size_t Width>
    void uncorrelateCovariance(size_t first)
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        Vector<Type, Width> diag_elements = self.slice<Width, Width>(first, first).diag();
        self.uncorrelateCovarianceSetVariance(first, diag_elements);
    }

    template <size_t Width>
    void uncorrelateCovarianceSetVariance(size_t first, const Vector<Type, Width> &vec)
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        // zero rows and columns
        self.slice<Width, M>(first, 0) = Type(0);
        self.slice<M, Width>(0, first) = Type(0);

        // set diagonals
        unsigned vec_idx = 0;
        for (size_t idx = first; idx < first+Width; idx++) {
            self(idx,idx) = vec(vec_idx);
            vec_idx ++;
        }
    }

    template <size_t Width>
    void uncorrelateCovarianceSetVariance(size_t first, Type val)
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        // zero rows and columns
        self.slice<Width, M>(first, 0) = Type(0);
        self.slice<M, Width>(0, first) = Type(0);

        // set diagonals
        for (size_t idx = first; idx < first+Width; idx++) {
            self(idx,idx) = val;
        }
    }

    // make block diagonal symmetric by taking the average of the two corresponding off diagonal values
    template <size_t Width>
    void makeBlockSymmetric(size_t first)
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        if(Width>1) {
            for (size_t row_idx = first+1; row_idx < first+Width; row_idx++) {
                for (size_t col_idx = first; col_idx < row_idx; col_idx++) {
                    Type tmp = (self(row_idx,col_idx) + self(col_idx,row_idx)) / Type(2);
                    self(row_idx,col_idx) = tmp;
                    self(col_idx,row_idx) = tmp;
                }
            }
        }
    }

    // make rows and columns symmetric by taking the average of the two corresponding off diagonal values
    template <size_t Width>
    void makeRowColSymmetric(size_t first)
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        self.makeBlockSymmetric<Width>(first);
        for (size_t row_idx = first; row_idx < first+Width; row_idx++) {
            for (size_t col_idx = 0; col_idx < first; col_idx++) {
                Type tmp = (self(row_idx,col_idx) + self(col_idx,row_idx)) / Type(2);
                self(row_idx,col_idx) = tmp;
                self(col_idx,row_idx) = tmp;
            }
            for (size_t col_idx = first+Width; col_idx < M; col_idx++) {
                Type tmp = (self(row_idx,col_idx) + self(col_idx,row_idx)) / Type(2);
                self(row_idx,col_idx) = tmp;
                self(col_idx,row_idx) = tmp;
            }
        }
    }

    // checks if block diagonal is symmetric
    template <size_t Width>
    bool isBlockSymmetric(size_t first, const Type eps = Type(1e-8f))
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        if(Width>1) {
            for (size_t row_idx = first+1; row_idx < first+Width; row_idx++) {
                for (size_t col_idx = first; col_idx < row_idx; col_idx++) {
                    if(!isEqualF(self(row_idx,col_idx), self(col_idx,row_idx), eps)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    // checks if rows and columns are symmetric
    template <size_t Width>
    bool isRowColSymmetric(size_t first, const Type eps = Type(1e-8f))
    {
        static_assert(Width <= M, "Width bigger than matrix");
        assert(first + Width <= M);

        SquareMatrix<Type, M> &self = *this;
        for (size_t row_idx = first; row_idx < first+Width; row_idx++) {
            for (size_t col_idx = 0; col_idx < first; col_idx++) {
                if(!isEqualF(self(row_idx,col_idx), self(col_idx,row_idx), eps)) {
                    return false;
                }
            }
            for (size_t col_idx = first+Width; col_idx < M; col_idx++) {
                if(!isEqualF(self(row_idx,col_idx), self(col_idx,row_idx), eps)) {
                    return false;
                }
            }
        }
        return self.isBlockSymmetric<Width>(first, eps);
    }

};

using SquareMatrix3f = SquareMatrix<float, 3>;
using SquareMatrix3d = SquareMatrix<double, 3>;

template<typename Type, size_t M>
SquareMatrix<Type, M> eye() {
    SquareMatrix<Type, M> m;
    m.setIdentity();
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> diag(Vector<Type, M> d) {
    SquareMatrix<Type, M> m;
    for (size_t i=0; i<M; i++) {
        m(i,i) = d(i);
    }
    return m;
}

template<typename Type, size_t M>
SquareMatrix<Type, M> expm(const Matrix<Type, M, M> & A, size_t order=5)
{
    SquareMatrix<Type, M> res;
    SquareMatrix<Type, M> A_pow = A;
    res.setIdentity();
    size_t i_factorial = 1;
    for (size_t i=1; i<=order; i++) {
        i_factorial *= i;
        res += A_pow / Type(i_factorial);
        A_pow *= A_pow;
    }

    return res;
}


/**
 * inverse based on LU factorization with partial pivotting
 */
template<typename Type, size_t M>
bool inv(const SquareMatrix<Type, M> & A, SquareMatrix<Type, M> & inv, size_t rank = M)
{
    SquareMatrix<Type, M> L;
    L.setIdentity();
    SquareMatrix<Type, M> U = A;
    SquareMatrix<Type, M> P;
    P.setIdentity();

    //printf("A:\n"); A.print();

    // for all diagonal elements
    for (size_t n = 0; n < rank; n++) {

        // if diagonal is zero, swap with row below
        if (fabs(U(n, n)) < Type(FLT_EPSILON)) {
            //printf("trying pivot for row %d\n",n);
            for (size_t i = n + 1; i < rank; i++) {

                //printf("\ttrying row %d\n",i);
                if (fabs(U(i, n)) > Type(FLT_EPSILON)) {
                    //printf("swapped %d\n",i);
                    U.swapRows(i, n);
                    P.swapRows(i, n);
                    L.swapRows(i, n);
                    L.swapCols(i, n);
                    break;
                }
            }
        }

#ifdef MATRIX_ASSERT
        //printf("A:\n"); A.print();
        //printf("U:\n"); U.print();
        //printf("P:\n"); P.print();
        //fflush(stdout);
        //ASSERT(fabs(U(n, n)) > 1e-8f);
#endif

        // failsafe, return zero matrix
        if (fabs(static_cast<float>(U(n, n))) < FLT_EPSILON) {
            return false;
        }

        // for all rows below diagonal
        for (size_t i = (n + 1); i < rank; i++) {
            L(i, n) = U(i, n) / U(n, n);

            // add i-th row and n-th row
            // multiplied by: -a(i,n)/a(n,n)
            for (size_t k = n; k < rank; k++) {
                U(i, k) -= L(i, n) * U(n, k);
            }
        }
    }

    //printf("L:\n"); L.print();
    //printf("U:\n"); U.print();

    // solve LY=P*I for Y by forward subst
    //SquareMatrix<Type, M> Y = P;

    // for all columns of Y
    for (size_t c = 0; c < rank; c++) {
        // for all rows of L
        for (size_t i = 0; i < rank; i++) {
            // for all columns of L
            for (size_t j = 0; j < i; j++) {
                // for all existing y
                // subtract the component they
                // contribute to the solution
                P(i, c) -= L(i, j) * P(j, c);
            }

            // divide by the factor
            // on current
            // term to be solved
            // Y(i,c) /= L(i,i);
            // but L(i,i) = 1.0
        }
    }

    //printf("Y:\n"); Y.print();

    // solve Ux=y for x by back subst
    //SquareMatrix<Type, M> X = Y;

    // for all columns of X
    for (size_t c = 0; c < rank; c++) {
        // for all rows of U
        for (size_t k = 0; k < rank; k++) {
            // have to go in reverse order
            size_t i = rank - 1 - k;

            // for all columns of U
            for (size_t j = i + 1; j < rank; j++) {
                // for all existing x
                // subtract the component they
                // contribute to the solution
                P(i, c) -= U(i, j) * P(j, c);
            }

            // divide by the factor
            // on current
            // term to be solved
            //
            // we know that U(i, i) != 0 from above
            P(i, c) /= U(i, i);
        }
    }

    //check sanity of results
    for (size_t i = 0; i < rank; i++) {
        for (size_t j = 0; j < rank; j++) {
            if (!is_finite(P(i,j))) {
                return false;
            }
        }
    }
    //printf("X:\n"); X.print();
    inv = P;
    return true;
}

template<typename Type>
bool inv(const SquareMatrix<Type, 2> & A, SquareMatrix<Type, 2> & inv)
{
    Type det = A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1);

    if(fabs(static_cast<float>(det)) < FLT_EPSILON || !is_finite(det)) {
        return false;
    }

    inv(0, 0) = A(1, 1);
    inv(1, 0) = -A(1, 0);
    inv(0, 1) = -A(0, 1);
    inv(1, 1) = A(0, 0);
    inv /= det;
    return true;
}

template<typename Type>
bool inv(const SquareMatrix<Type, 3> & A, SquareMatrix<Type, 3> & inv)
{
    Type det = A(0, 0) * (A(1, 1) * A(2, 2) - A(2, 1) * A(1, 2)) -
               A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0)) +
               A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));

    if(fabs(static_cast<float>(det)) < FLT_EPSILON || !is_finite(det)) {
        return false;
    }

    inv(0, 0) = A(1, 1) * A(2, 2) - A(2, 1) * A(1, 2);
    inv(0, 1) = A(0, 2) * A(2, 1) - A(0, 1) * A(2, 2);
    inv(0, 2) = A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1);
    inv(1, 0) = A(1, 2) * A(2, 0) - A(1, 0) * A(2, 2);
    inv(1, 1) = A(0, 0) * A(2, 2) - A(0, 2) * A(2, 0);
    inv(1, 2) = A(1, 0) * A(0, 2) - A(0, 0) * A(1, 2);
    inv(2, 0) = A(1, 0) * A(2, 1) - A(2, 0) * A(1, 1);
    inv(2, 1) = A(2, 0) * A(0, 1) - A(0, 0) * A(2, 1);
    inv(2, 2) = A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1);
    inv /= det;
    return true;
}

/**
 * inverse based on LU factorization with partial pivotting
 */
template<typename Type, size_t M>
SquareMatrix<Type, M> inv(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> i;
    if (inv(A, i)) {
        return i;
    } else {
        i.setZero();
        return i;
    }
}

/**
 * cholesky decomposition
 *
 * Note: A must be positive definite
 */
template<typename Type, size_t M>
SquareMatrix <Type, M> cholesky(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> L;
    for (size_t j = 0; j < M; j++) {
        for (size_t i = j; i < M; i++) {
            if (i==j) {
                float sum = 0;
                for (size_t k = 0; k < j; k++) {
                    sum += L(j, k)*L(j, k);
                }
                Type res = A(j, j) - sum;
                if (res <= 0) {
                    L(j, j) = 0;
                } else {
                    L(j, j) = sqrt(res);
                }
            } else {
                float sum = 0;
                for (size_t k = 0; k < j; k++) {
                    sum += L(i, k)*L(j, k);
                }
                if (L(j, j) <= 0) {
                    L(i, j) = 0;
                } else {
                    L(i, j) = (A(i, j) - sum)/L(j, j);
                }
            }
        }
    }
    return L;
}

/**
 * cholesky inverse
 *
 * TODO: Check if gaussian elimination jumps straight to back-substitution
 * for L or we need to do it manually. Will impact speed otherwise.
 */
template<typename Type, size_t M>
SquareMatrix <Type, M> choleskyInv(const SquareMatrix<Type, M> & A)
{
    SquareMatrix<Type, M> L_inv = inv(cholesky(A));
    return L_inv.T()*L_inv;
}

using Matrix3f = SquareMatrix<float, 3>;
using Matrix3d = SquareMatrix<double, 3>;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
