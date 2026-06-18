#include <rl_tools/operations/cpu.h>


#include <gtest/gtest.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


TEST(RL_TOOLS_TEST_CONTAINER, SLICE){
    using DEVICE = rlt::devices::DefaultCPU;
    using DTYPE = float;
    DEVICE device;
    rlt::Matrix<rlt::matrix::Specification<float, typename DEVICE::index_t, 3, 3>> m{};
    rlt::malloc(device, m);
    rlt::set(m, 0, 0, 1);
    rlt::set(m, 0, 1, 2);
    rlt::set(m, 0, 2, 3);
    rlt::set(m, 1, 0, 4);
    rlt::set(m, 1, 1, 5);
    rlt::set(m, 1, 2, 6);
    rlt::set(m, 2, 0, 7);
    rlt::set(m, 2, 1, 8);
    rlt::set(m, 2, 2, 9);
    rlt::print(device, m);
    auto m2 = rlt::view<DEVICE, decltype(m)::SPEC, 2, 2>(device, m, 0, 1);
    rlt::print(device, m2);
    ASSERT_FLOAT_EQ(rlt::get(m2, 0, 0), 2);
    ASSERT_FLOAT_EQ(rlt::get(m2, 0, 1), 3);
    ASSERT_FLOAT_EQ(rlt::get(m2, 1, 0), 5);
    ASSERT_FLOAT_EQ(rlt::get(m2, 1, 1), 6);

    auto m3 = rlt::view_transpose(device, m);
    std::cout << "m3 = transpose(m): " << std::endl;
    rlt::print(device, m3);
    rlt::free(device, m);

    rlt::Matrix<rlt::matrix::Specification<float, typename DEVICE::index_t, 17, 15>> m4;
    rlt::malloc(device, m4);
    //init with random data
    for(typename DEVICE::index_t row_i = 0; row_i < decltype(m4)::ROWS; row_i++){
        for(typename DEVICE::index_t col_i = 0; col_i < decltype(m4)::COLS; col_i++){
            rlt::set(m4, row_i, col_i, row_i * col_i);
        }
    }
    auto m5 = rlt::view<DEVICE, decltype(m4)::SPEC, 5, 5>(device, m4, 3, 4);
    std::cout << "m5 = (5x5 slice of m4): " << std::endl;
    rlt::print(device, m5);
    for(typename DEVICE::index_t row_i = 0; row_i < decltype(m5)::ROWS; row_i++){
        for(typename DEVICE::index_t col_i = 0; col_i < decltype(m5)::COLS; col_i++){
            ASSERT_FLOAT_EQ(rlt::get(m5, row_i, col_i), (row_i + 3) * (col_i + 4));
        }
    }
    rlt::free(device, m4);

}
template <int ROWS, int COLS, int ROW_PITCH, int COL_PITCH, int VIEW_ROWS, int VIEW_COLS>
void test_view(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::Fixed<TI, ROW_PITCH, COL_PITCH>>> m;
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::RowMajorAlignment<TI, 1>>> m_dense;
    rlt::malloc(device, m);
    rlt::malloc(device, m_dense);
    rlt::randn(device, m, rng);
    rlt::copy(device, device, m, m_dense);

    for(TI row_i = 0; row_i < ROWS-VIEW_ROWS; row_i++){
        for(TI col_i = 0; col_i < COLS-VIEW_COLS; col_i++){
            auto view = rlt::view(device, m, rlt::matrix::ViewSpec<VIEW_ROWS, VIEW_COLS>(), row_i, col_i);
            auto view_dense = rlt::view(device, m_dense, rlt::matrix::ViewSpec<VIEW_ROWS, VIEW_COLS>(), row_i, col_i);
            auto abs_diff = rlt::abs_diff(device, view, view_dense);
            ASSERT_FLOAT_EQ(abs_diff, 0);
        }
    }

    rlt::free(device, m);
    rlt::free(device, m_dense);

}
TEST(RL_TOOLS_TEST_CONTAINER, VIEW) {
    test_view<10, 10, 10, 1, 5, 5>();
    test_view<10, 10, 20, 2, 5, 5>();
    test_view<15, 13, 100, 3, 3, 2>();
    test_view<15, 13, 3, 100, 1, 1>();
    test_view<15, 13, 3, 100, 10, 1>();
    test_view<15, 13, 3, 100, 1, 10>();
}

template <int ROWS, int COLS, int ROW_PITCH, int COL_PITCH>
void test_view_col(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::Fixed<TI, ROW_PITCH, COL_PITCH>>> m;
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::RowMajorAlignment<TI, 1>>> m_dense;
    rlt::malloc(device, m);
    rlt::malloc(device, m_dense);
    rlt::randn(device, m, rng);
    rlt::copy(device, device, m, m_dense);

    for(TI row_i = 0; row_i < ROWS; row_i++){
        auto row_m = rlt::row(device, m, row_i);
        auto row_m_dense = rlt::row(device, m_dense, row_i);
        auto abs_diff = rlt::abs_diff(device, row_m, row_m_dense);
        ASSERT_FLOAT_EQ(abs_diff, 0);
    }

    for(TI col_i = 0; col_i < COLS; col_i++){
        auto col_m = rlt::col(device, m, col_i);
        auto col_m_dense = rlt::col(device, m_dense, col_i);
        auto abs_diff = rlt::abs_diff(device, col_m, col_m_dense);
        ASSERT_FLOAT_EQ(abs_diff, 0);
    }
    rlt::free(device, m);
    rlt::free(device, m_dense);

}

TEST(RL_TOOLS_TEST_CONTAINER, VIEW_ROW_COL) {
    test_view_col<10, 10, 10, 1>();
    test_view_col<10, 10, 20, 2>();
    test_view_col<15, 13, 100, 3>();
    test_view_col<15, 13, 3, 100>();
}

template <int ROWS, int COLS, int ROW_PITCH, int COL_PITCH>
void test_is_nan(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::Fixed<TI, ROW_PITCH, COL_PITCH>>> m;
    rlt::malloc(device, m);
    rlt::randn(device, m, rng);

    bool is_nan_m = rlt::is_nan(device, m);
    ASSERT_TRUE(!is_nan_m);

    set(m, ROWS/2, COLS/2, std::numeric_limits<T>::quiet_NaN());
    is_nan_m = rlt::is_nan(device, m);
    ASSERT_TRUE(is_nan_m);

    rlt::free(device, m);
}

TEST(RL_TOOLS_TEST_CONTAINER, IS_NAN) {
    test_is_nan<10, 10, 10, 1>();
    test_is_nan<10, 10, 20, 2>();
    test_is_nan<15, 13, 100, 3>();
    test_is_nan<15, 13, 3, 100>();
}
template <int ROWS, int COLS, int ROW_PITCH, int COL_PITCH>
void test_is_finite(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS, true, rlt::matrix::layouts::Fixed<TI, ROW_PITCH, COL_PITCH>>> m;
    rlt::malloc(device, m);
    rlt::randn(device, m, rng);

    bool is_finite = rlt::is_finite(device, m);
    ASSERT_TRUE(is_finite);

    set(m, ROWS/2, COLS/2, std::numeric_limits<T>::infinity());
    std::cout << "setting infinity: " << std::endl;
    rlt::print(device, m);
    is_finite = rlt::is_finite(device, m);
    ASSERT_TRUE(!is_finite);

    set(m, ROWS/2, COLS/2, std::numeric_limits<T>::quiet_NaN());
    std::cout << "setting quiet nan: " << std::endl;
    rlt::print(device, m);
    is_finite = rlt::is_finite(device, m);
    ASSERT_TRUE(!is_finite);

    rlt::free(device, m);
}

TEST(RL_TOOLS_TEST_CONTAINER, IS_FINITE){
    test_is_finite<10, 10, 10, 1>();
    test_is_finite<10, 10, 20, 2>();
    test_is_finite<15, 13, 100, 3>();
    test_is_finite<15, 13, 3, 100>();
}

TEST(RL_TOOLS_TEST_CONTAINER, WRAP){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    T test[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    auto m = rlt::wrap<DEVICE, T, DIM>(device, test);
    rlt::print(device, m);
    for(TI i = 0; i < DIM; i++){
        ASSERT_FLOAT_EQ(get(m, 0, i), test[i]);
    }
}

TEST(RL_TOOLS_TEST_CONTAINER, MIN_DETERMINISTIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    T test[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    auto m = rlt::wrap<DEVICE, T, DIM>(device, test);
    rlt::print(device, m);
    T min = rlt::min(device, m);
    ASSERT_FLOAT_EQ(min, 1);
}

TEST(RL_TOOLS_TEST_CONTAINER, MAX_DETERMINISTIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    T test[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    auto m = rlt::wrap<DEVICE, T, DIM>(device, test);
    rlt::print(device, m);
    T max = rlt::max(device, m);
    ASSERT_FLOAT_EQ(max, 11);
}

template <int ROWS, int COLS>
void test_max_stochastic(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS>> m;
    rlt::malloc(device, m);
    for(TI test_i = 0; test_i < 10; test_i++){
        rlt::randn(device, m, rng);
        T max = rlt::max(device, m);
        for(TI row_i = 0; row_i < ROWS; row_i++){
            for(TI col_i = 0; col_i < COLS; col_i++){
                ASSERT_TRUE(get(m, row_i, col_i) <= max);
            }
        }
    }
}
TEST(RL_TOOLS_TEST_CONTAINER, MAX_STOCHASTIC){
    test_max_stochastic<10, 10>();
    test_max_stochastic<10, 1000>();
    test_max_stochastic<1, 1>();
    test_max_stochastic<1, 10>();
    test_max_stochastic<10, 1>();
}

TEST(RL_TOOLS_TEST_CONTAINER, ARGMAX_DETERMINISTIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    {
        T test[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        auto m = rlt::wrap<DEVICE, T, DIM>(device, test);
        rlt::print(device, m);
        TI am = rlt::argmax_row(device, m);
        ASSERT_FLOAT_EQ(am, 10);
    }
    {
        T test[] = {1, 2, 3, 4, 50, 6, 7, 8, 9, 10, 11};
        auto m = rlt::wrap<DEVICE, T, DIM>(device, test);
        rlt::print(device, m);
        TI am = rlt::argmax_row(device, m);
        ASSERT_FLOAT_EQ(am, 4);
    }
}

template <int ROWS, int COLS>
void test_argmax_stochastic(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    constexpr int DIM = 11;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS>> m;
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, 1>> am;
    rlt::malloc(device, m);
    rlt::malloc(device, am);
    for(TI test_i = 0; test_i < 10; test_i++){
        rlt::randn(device, m, rng);
        rlt::argmax_row_wise(device, m, am);
        for(TI row_i = 0; row_i < ROWS; row_i++){
            T row_max = get(m, row_i, get(am, row_i, 0));
            for(TI col_i = 0; col_i < COLS; col_i++){
                if(!(get(m, row_i, col_i) <= row_max)){
                    rlt::print(device, m);
                }
                ASSERT_TRUE(get(m, row_i, col_i) <= row_max);
            }
        }
    }
}
TEST(RL_TOOLS_TEST_CONTAINER, ARGMAX_STOCHASTIC){
    test_argmax_stochastic<10, 10>();
    test_argmax_stochastic<10, 1000>();
    test_argmax_stochastic<1, 1>();
    test_argmax_stochastic<1, 10>();
    test_argmax_stochastic<10, 1>();
}


TEST(RL_TOOLS_TEST_CONTAINER, MATRIX_MULTIPLICATION_GENERIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 2, 2>> A, B, C, C_target;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    set(A, 0, 0, -0.259093);
    set(A, 0, 1, -1.498961);
    set(A, 1, 0, +0.119264);
    set(A, 1, 1, +0.458181);

    set(B, 0, 0, +0.394975);
    set(B, 0, 1, +0.044197);
    set(B, 1, 0, -0.636256);
    set(B, 1, 1, +1.731264);

    set(C_target, 0, 0, -0.259093 * +0.394975 + -1.498961 * -0.636256);
    set(C_target, 0, 1, -0.259093 * +0.044197 + -1.498961 * +1.731264);
    set(C_target, 1, 0, +0.119264 * +0.394975 + +0.458181 * -0.636256);
    set(C_target, 1, 1, +0.119264 * +0.044197 + +0.458181 * +1.731264);
    rlt::print(device, C_target);

    rlt::multiply(device, A, B, C);
    rlt::print(device, C);
    auto diff = rlt::abs_diff(device, C_target, C);
    std::cout << "Matrix mul diff: " << diff << std::endl;
    ASSERT_TRUE(diff < 1e-6);
}

#ifdef RL_TOOLS_BACKEND_ENABLE_MKL
#define RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION
#include <rl_tools/operations/cpu_mkl.h>
TEST(RL_TOOLS_TEST_CONTAINER, MATRIX_MULTIPLICATION_MKL){
    using DEVICE = rlt::devices::DefaultCPU_MKL;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 2, 2>> A, B, C, C_target;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    set(A, 0, 0, -0.259093);
    set(A, 0, 1, -1.498961);
    set(A, 1, 0, +0.119264);
    set(A, 1, 1, +0.458181);

    set(B, 0, 0, +0.394975);
    set(B, 0, 1, +0.044197);
    set(B, 1, 0, -0.636256);
    set(B, 1, 1, +1.731264);

    set(C_target, 0, 0, -0.259093 * +0.394975 + -1.498961 * -0.636256);
    set(C_target, 0, 1, -0.259093 * +0.044197 + -1.498961 * +1.731264);
    set(C_target, 1, 0, +0.119264 * +0.394975 + +0.458181 * -0.636256);
    set(C_target, 1, 1, +0.119264 * +0.044197 + +0.458181 * +1.731264);
    rlt::print(device, C_target);

    rlt::multiply(device, A, B, C);
    rlt::print(device, C);
    auto diff = rlt::abs_diff(device, C_target, C);
    std::cout << "Matrix mul diff: " << diff << std::endl;
    ASSERT_TRUE(diff < 1e-6);
}

template <typename T, typename TI, TI M, TI K, TI N>
void test_matrix_multiplication_mkl_generic(){
    using DEVICE_MKL = rlt::devices::DefaultCPU_MKL;
    using DEVICE_GENERIC = rlt::devices::DefaultCPU;
    DEVICE_MKL device_mkl;
    DEVICE_GENERIC device;
    DEVICE_GENERIC::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, K>> A;
    rlt::Matrix<rlt::matrix::Specification<T, TI, K, N>> B;
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, N>> C, C_target;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    rlt::randn(device, A, rng);
    rlt::randn(device, B, rng);

    rlt::multiply(device, A, B, C_target);
    rlt::multiply(device_mkl, A, B, C);
    std::cout << "A (" << M << "x" << K << "), B (" << K << "x" << N << "), C (" << M << "x" << N << "):" << std::endl;
    std::cout << "target:" << std::endl;
    rlt::print(device, C_target);
    std::cout << "real:" << std::endl;
    rlt::print(device, C);
    T diff = rlt::abs_diff(device, C_target, C);
    T diff_per_element = diff / (M * N);
    std::cout << "Matrix mul diff: " << diff << " per element: " << diff_per_element << std::endl;
    if(rlt::utils::typing::is_same_v<T, float>){
        ASSERT_LT(diff_per_element, 1e-5);
    }else{
        ASSERT_LT(diff_per_element, 1e-10);
    }
}
TEST(RL_TOOLS_TEST_CONTAINER, MATRIX_MULTIPLICATION_MKL_GENERIC){
    test_matrix_multiplication_mkl_generic<float, int, 1, 1, 1>();
    test_matrix_multiplication_mkl_generic<float, int, 5, 2, 2>();
    test_matrix_multiplication_mkl_generic<float, int, 5, 10, 10>();
    test_matrix_multiplication_mkl_generic<float, int, 45, 10, 1000>();
    test_matrix_multiplication_mkl_generic<float, int, 35, 1, 1>();
    test_matrix_multiplication_mkl_generic<float, int, 52, 1, 10>();
    test_matrix_multiplication_mkl_generic<float, int, 35, 10, 1>();
    test_matrix_multiplication_mkl_generic<double, int, 1, 1, 1>();
    test_matrix_multiplication_mkl_generic<double, int, 5, 2, 2>();
    test_matrix_multiplication_mkl_generic<double, int, 15, 10, 10>();
    test_matrix_multiplication_mkl_generic<double, int, 5, 10, 1000>();
    test_matrix_multiplication_mkl_generic<double, int, 115, 1, 1>();
    test_matrix_multiplication_mkl_generic<double, int, 5, 1, 10>();
    test_matrix_multiplication_mkl_generic<double, int, 55, 10, 1>();
}
#endif

#ifdef RL_TOOLS_BACKEND_ENABLE_OPENBLAS
#define RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION
#include <rl_tools/operations/cpu_openblas.h>
TEST(RL_TOOLS_TEST_CONTAINER, MATRIX_MULTIPLICATION_OPENBLAS) {
    using DEVICE = rlt::devices::DefaultCPU_OPENBLAS;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 2, 2>> A, B, C, C_target;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    set(A, 0, 0, -0.259093);
    set(A, 0, 1, -1.498961);
    set(A, 1, 0, +0.119264);
    set(A, 1, 1, +0.458181);

    set(B, 0, 0, +0.394975);
    set(B, 0, 1, +0.044197);
    set(B, 1, 0, -0.636256);
    set(B, 1, 1, +1.731264);

    set(C_target, 0, 0, -0.259093 * +0.394975 + -1.498961 * -0.636256);
    set(C_target, 0, 1, -0.259093 * +0.044197 + -1.498961 * +1.731264);
    set(C_target, 1, 0, +0.119264 * +0.394975 + +0.458181 * -0.636256);
    set(C_target, 1, 1, +0.119264 * +0.044197 + +0.458181 * +1.731264);
    rlt::print(device, C_target);

    rlt::multiply(device, A, B, C);
    rlt::print(device, C);
    auto diff = rlt::abs_diff(device, C_target, C);
    std::cout << "Matrix mul diff: " << diff << std::endl;
    ASSERT_TRUE(diff < 1e-6);
}
#endif

TEST(RL_TOOLS_TEST_CONTAINER, RESHAPE) {
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 10, 6, false>> m;
    using M = decltype(m);
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    rlt::randn(device, m, rng);

    rlt::print(device, m);
    std::cout << "Before: " << std::endl;
    std::cout << "Shape: " << M::SPEC::ROWS << " x " << M::SPEC::COLS << std::endl;
    std::cout << "Pitch: " << M::SPEC::ROW_PITCH << " x " << M::SPEC::COL_PITCH << std::endl;

    auto reshaped = rlt::reshape<20, 3>(device, m);
    using R = decltype(reshaped);
    std::cout << "After: " << std::endl;
    std::cout << "Shape: " << R::SPEC::ROWS << " x " << R::SPEC::COLS << std::endl;
    std::cout << "Pitch: " << R::SPEC::ROW_PITCH << " x " << R::SPEC::COL_PITCH << std::endl;
    rlt::print(device, reshaped);

    for(TI row_i = 0; row_i < M::SPEC::ROWS; row_i++){
        for(TI col_i = 0; col_i < M::SPEC::COLS; col_i++){
            TI new_row = (row_i * M::SPEC::COLS + col_i) / R::SPEC::COLS;
            TI new_col = (row_i * M::SPEC::COLS + col_i) % R::SPEC::COLS;
            ASSERT_FLOAT_EQ(rlt::get(m, row_i, col_i), rlt::get(reshaped, new_row, new_col));
        }
    }

}
