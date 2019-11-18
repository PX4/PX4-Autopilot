/**
 * @file pseudoinverse.hxx
 *
 * Implementation of matrix pseudo inverse
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */



/**
 * Full rank Cholesky factorization of A
 */
template<typename Type>
void fullRankCholeskyTolerance(Type &tol)
{
    tol /= 1000000000;
}

template<> inline
void fullRankCholeskyTolerance<double>(double &tol)
{
    tol /= 1000000000000000000.0;
}

template<typename Type, size_t N>
SquareMatrix<Type, N> fullRankCholesky(const SquareMatrix<Type, N> & A,
                               size_t& rank)
{
    // Compute
    // dA = np.diag(A)
    // tol = np.min(dA[dA > 0]) * 1e-9
    Vector<Type, N> d = A.diag();
    Type tol = d.max();
    for (size_t k = 0; k < N; k++) {
        if ((d(k) > 0) && (d(k) < tol)) {
            tol = d(k);
        }
    }
    fullRankCholeskyTolerance<Type>(tol);

    Matrix<Type, N, N> L;

    size_t r = 0;
    for (size_t k = 0; k < N; k++) {

        if (r == 0) {
            for (size_t i = k; i < N; i++) {
                L(i, r) = A(i, k);
            }

        } else {
            for (size_t i = k; i < N; i++) {
                // Compute LL = L[k:n, :r] * L[k, :r].T
                Type LL = Type();
                for (size_t j = 0; j < r; j++) {
                    LL += L(i, j) * L(k, j);
                }
                L(i, r) = A(i, k) - LL;
            }
        }

        if (L(k, r) > tol) {
            L(k, r) = sqrt(L(k, r));

            if (k < N - 1) {
                for (size_t i = k + 1; i < N; i++) {
                    L(i, r) = L(i, r) / L(k, r);
                }
            }

            r = r + 1;
        }
    }

    // Return rank
    rank = r;

    return L;
}

template< typename Type, size_t M, size_t N, size_t R>
class GeninvImpl
{
public:
    static Matrix<Type, N, M> genInvUnderdetermined(const Matrix<Type, M, N> & G, const Matrix<Type, M, M> & L, size_t rank)
    {
        if (rank < R) {
            // Recursive call
            return GeninvImpl<Type, M, N, R - 1>::genInvUnderdetermined(G, L, rank);

        } else if (rank > R) {
            // Error
            return Matrix<Type, N, M>();

        } else {
            // R == rank
            Matrix<Type, M, R> LL = L. template slice<M, R>(0, 0);
            SquareMatrix<Type, R> X = inv(SquareMatrix<Type, R>(LL.transpose() * LL));
            return G.transpose() * LL * X * X * LL.transpose();

        }
    }

    static Matrix<Type, N, M> genInvOverdetermined(const Matrix<Type, M, N> & G, const Matrix<Type, N, N> & L, size_t rank)
    {
        if (rank < R) {
            // Recursive call
            return GeninvImpl<Type, M, N, R - 1>::genInvOverdetermined(G, L, rank);

        } else if (rank > R) {
            // Error
            return Matrix<Type, N, M>();

        } else {
            // R == rank
            Matrix<Type, N, R> LL = L. template slice<N, R>(0, 0);
            SquareMatrix<Type, R> X = inv(SquareMatrix<Type, R>(LL.transpose() * LL));
            return LL * X * X * LL.transpose() * G.transpose();

        }
    }
};

// Partial template specialisation for R==0, allows to stop recursion in genInvUnderdetermined and genInvOverdetermined
template< typename Type, size_t M, size_t N>
class GeninvImpl<Type, M, N, 0>
{
public:
    static Matrix<Type, N, M> genInvUnderdetermined(const Matrix<Type, M, N> & G, const Matrix<Type, M, M> & L, size_t rank)
    {
        return Matrix<Type, N, M>();
    }

    static Matrix<Type, N, M> genInvOverdetermined(const Matrix<Type, M, N> & G, const Matrix<Type, N, N> & L, size_t rank)
    {
        return Matrix<Type, N, M>();
    }
};
