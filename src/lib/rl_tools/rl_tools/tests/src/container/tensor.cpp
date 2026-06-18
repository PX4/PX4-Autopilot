#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <chrono>

#include <rl_tools/operations/cpu.h>
#include <rl_tools/containers/tensor/tensor.h>
#include <rl_tools/containers/tensor/operations_generic.h>
#include <rl_tools/containers/tensor/operations_cpu.h>
namespace rlt = rl_tools;

constexpr double EPSILON = 1e-8;


template <typename INPUT>
void test_shape_operations(typename INPUT::TI length){
    ASSERT_TRUE(length == rlt::length(INPUT{}));
    using APPEND = rlt::tensor::Append<INPUT, 5>;
    static_assert((rlt::length(INPUT{}) + 1) == rlt::length(APPEND{}));
    using PREPEND = rlt::tensor::Prepend<INPUT, 10>;
    static_assert((rlt::length(INPUT{}) + 1) == rlt::length(PREPEND{}));

    if constexpr(rlt::length(INPUT{}) > 1){
        using POP_FRONT = rlt::tensor::PopFront<INPUT>;
        static_assert(rlt::length(INPUT{}) == (rlt::length(POP_FRONT{}) + 1));
        using POP_BACK = rlt::tensor::PopBack<INPUT>;
        static_assert(rlt::length(INPUT{}) == (rlt::length(POP_BACK{}) + 1));

        if constexpr (rlt::length(INPUT{}) >= 3){
            static_assert(rlt::get<0>(POP_FRONT{}) == rlt::get<1>(INPUT{}));
            static_assert(rlt::get<1>(POP_FRONT{}) == rlt::get<2>(INPUT{}));
        }
        if constexpr(rlt::length(INPUT{}) >= 3){
            static_assert(rlt::get<0>(POP_BACK{}) == rlt::get<0>(INPUT{}));
            static_assert(rlt::get<1>(POP_BACK{}) == rlt::get<1>(INPUT{}));
        }
        {
            using REMOVE = rlt::tensor::Remove<INPUT, 0>;
            static_assert(rlt::get<0>(REMOVE{}) == rlt::get<1>(INPUT{}));
            static_assert(rlt::length(INPUT{}) - 1 == rlt::length(REMOVE{}));
        }
        {
            using REMOVE = rlt::tensor::Remove<INPUT, 1>;
            static_assert(rlt::length(INPUT{}) - 1 == rlt::length(REMOVE{}));
            static_assert(rlt::get<0>(REMOVE{}) == rlt::get<0>(REMOVE{}));
        }
        if constexpr(rlt::length(INPUT{}) == 2){
            using REMOVE = rlt::tensor::Remove<INPUT, 1>;
            static_assert(rlt::length(REMOVE{}) == 1);
        }
        {
            using INSERT = rlt::tensor::Insert<INPUT, 1337, 0>;
            static_assert(rlt::get<0>(INSERT{}) == 1337);
            static_assert(rlt::length(INPUT{}) + 1 == rlt::length(INSERT{}));
            static_assert(rlt::get<1>(INSERT{}) == rlt::get<0>(INPUT{}));
            static_assert(rlt::get<rlt::length(INPUT{})>(INSERT{}) == rlt::get<rlt::length(INPUT{})-1>(INPUT{}));
        }
        {
            using INSERT = rlt::tensor::Insert<INPUT, 1337, 1>;
            static_assert(rlt::length(INPUT{}) + 1 == rlt::length(INSERT{}));
            static_assert(rlt::get<0>(INSERT{}) == rlt::get<0>(INPUT{}));
            static_assert(rlt::get<1>(INSERT{}) == 1337);
            static_assert(rlt::get<rlt::length(INPUT{})>(INSERT{}) == rlt::get<rlt::length(INPUT{})-1>(INPUT{}));
        }
    }


    if constexpr(rlt::length(INPUT{}) >= 3){
        {
            using REPLACE = rlt::tensor::Replace<INPUT, 10, 0>;
            static_assert(rlt::get<0>(REPLACE{}) == 10);
            static_assert(rlt::get<1>(REPLACE{}) == rlt::get<1>(INPUT{}));
            static_assert(rlt::get<2>(REPLACE{}) == rlt::get<2>(INPUT{}));
        }
        {
            using REPLACE = rlt::tensor::Replace<INPUT, 10, 1>;
            static_assert(rlt::get<1>(REPLACE{}) == 10);
            static_assert(rlt::get<0>(REPLACE{}) == rlt::get<0>(INPUT{}));
            static_assert(rlt::get<2>(REPLACE{}) == rlt::get<2>(INPUT{}));
        }
        {
            using REPLACE = rlt::tensor::Replace<INPUT, 10, 2>;
            static_assert(rlt::get<2>(REPLACE{}) == 10);
            static_assert(rlt::get<0>(REPLACE{}) == rlt::get<0>(INPUT{}));
            static_assert(rlt::get<1>(REPLACE{}) == rlt::get<1>(INPUT{}));
        }
    }
    {

        using REPLACE = rlt::tensor::Replace<INPUT, 10, 0>;
        static_assert(rlt::get<0>(REPLACE{}) == 10);
    }
    {
        using REPLACE = rlt::tensor::Replace<INPUT, 1337, rlt::length(INPUT{})-1>;
        static_assert(rlt::get<rlt::length(INPUT{})-1>(REPLACE{}) == 1337);
    }

    if constexpr(rlt::length(INPUT{}) == 3){
        using PRODUCT = rlt::tensor::CumulativeProduct<INPUT>;
        ASSERT_TRUE(rlt::get<0>(PRODUCT{}) == rlt::get<0>(INPUT{}) * rlt::get<1>(INPUT{}) * rlt::get<2>(INPUT{}));
        ASSERT_TRUE(rlt::get<1>(PRODUCT{}) == rlt::get<1>(INPUT{}) * rlt::get<2>(INPUT{}));
        ASSERT_TRUE(rlt::get<2>(PRODUCT{}) == rlt::get<2>(INPUT{}));
    }
}

TEST(RL_TOOLS_TENSOR_TEST, SHAPE_OPERATIONS){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using TEST_SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    using TEST_SPEC = rlt::tensor::Specification<T, TI, TEST_SHAPE>;
    static_assert(TEST_SPEC::SIZE == 2*3*4);

    using GET_LAST_TEST_SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    static_assert(rlt::get_last(GET_LAST_TEST_SHAPE{}) == 4);
    test_shape_operations<rlt::tensor::Shape<TI, 2, 3, 4>>(3);
    test_shape_operations<rlt::tensor::Shape<TI, 2, 3>>(2);
    test_shape_operations<rlt::tensor::Shape<TI, 2>>(1);
    test_shape_operations<rlt::tensor::Shape<TI, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0>>(10);
}
TEST(RL_TOOLS_TENSOR_TEST, SHAPE_OPERATIONS2){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    {
        using SHAPE = rlt::tensor::Shape<TI>;
        using INSERT = rlt::tensor::Insert<SHAPE, 1337, 0>;
        static_assert(rlt::get<0>(INSERT{}) == 1337);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI>;
        using APPEND = rlt::tensor::Append<SHAPE, 1337>;
        using INSERT = rlt::tensor::Insert<APPEND, 1338, 0>;
        static_assert(rlt::get<0>(INSERT{}) == 1338);
        static_assert(rlt::get<1>(INSERT{}) == 1337);
        static_assert(rlt::length(SHAPE{}) == 0);
        static_assert(rlt::length(APPEND{}) == 1);
        static_assert(rlt::length(INSERT{}) == 2);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
        using INSERT = rlt::tensor::Insert<SHAPE, 1337, 0>;
        static_assert(rlt::get<0>(INSERT{}) == 1337);
        static_assert(rlt::get<1>(INSERT{}) == 2);
        static_assert(rlt::get<2>(INSERT{}) == 3);
        static_assert(rlt::get<3>(INSERT{}) == 4);
        static_assert(rlt::length(SHAPE{}) == 3);
        static_assert(rlt::length(INSERT{}) == 4);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2>;
        static_assert(rlt::utils::typing::is_same_v<SHAPE::NEXT_ELEMENT::NEXT_ELEMENT, rlt::tensor::FinalElement>);
        using POP = rlt::tensor::PopFront<SHAPE>;
        using POP_BACK = rlt::tensor::PopBack<SHAPE>;
        static_assert(rlt::length(POP{}) == 0);
        static_assert(rlt::length(POP_BACK{}) == 0);
        using REMOVE = rlt::tensor::Remove<SHAPE, 0>;
        static_assert(rlt::length(SHAPE{}) == 1);
        static_assert(rlt::length(REMOVE{}) == 0);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4, 5>;
        static_assert(rlt::length(SHAPE{}) == 4);
        static_assert(rlt::get<0>(SHAPE{}) == 2);
        static_assert(rlt::get<1>(SHAPE{}) == 3);
        static_assert(rlt::get<2>(SHAPE{}) == 4);
        static_assert(rlt::get<3>(SHAPE{}) == 5);
        using POP = rlt::tensor::PopFront<SHAPE>;
        static_assert(rlt::length(POP{}) == 3);
        static_assert(rlt::get<0>(POP{}) == 3);
        static_assert(rlt::get<1>(POP{}) == 4);
        static_assert(rlt::get<2>(POP{}) == 5);
        using POP_BACK = rlt::tensor::PopBack<POP>;
        static_assert(rlt::length(POP_BACK{}) == 2);
        static_assert(rlt::get<0>(POP_BACK{}) == 3);
        static_assert(rlt::get<1>(POP_BACK{}) == 4);
        using APPEND = rlt::tensor::Append<POP_BACK, 6>;
        using REMOVE = rlt::tensor::Remove<APPEND, 0>;
        static_assert(rlt::length(REMOVE{}) == 2);
        static_assert(rlt::get<0>(REMOVE{}) == 4);
        static_assert(rlt::get<1>(REMOVE{}) == 6);
        using INSERT = rlt::tensor::Insert<REMOVE, 1337, 0>;
        static_assert(rlt::length(INSERT{}) == 3);
        static_assert(rlt::get<0>(INSERT{}) == 1337);
        static_assert(rlt::get<1>(INSERT{}) == 4);
        static_assert(rlt::get<2>(INSERT{}) == 6);
        using REPLACE = rlt::tensor::Replace<INSERT, 1338, 0>;
        static_assert(rlt::length(REPLACE{}) == 3);
        static_assert(rlt::get<0>(REPLACE{}) == 1338);
        static_assert(rlt::get<1>(REPLACE{}) == 4);
        static_assert(rlt::get<2>(REPLACE{}) == 6);
        using REPLACE2 = rlt::tensor::Replace<REPLACE, 1339, rlt::length(REPLACE{}) - 1>;
        static_assert(rlt::length(REPLACE2{}) == 3);
        static_assert(rlt::get<0>(REPLACE2{}) == 1338);
        static_assert(rlt::get<1>(REPLACE2{}) == 4);
        static_assert(rlt::get<2>(REPLACE2{}) == 1339);
    }
}
template <typename SHAPE, typename STRIDE>
bool test_row_major(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    std::cout << "Shape: \n";
    rlt::print(device, SHAPE{});
    std::cout << "Stride: \n";
    rlt::print(device, STRIDE{});

    return rlt::tensor::generalized_row_major<SHAPE, STRIDE>();
}
TEST(RL_TOOLS_TENSOR_TEST, ROW_MAJOR_TEST){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
        using STRIDE = rlt::tensor::Stride<TI, 13, 4, 1>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2, 4>;
        using STRIDE = rlt::tensor::Stride<TI, 13, 4, 1>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2, 2>;
        using STRIDE = rlt::tensor::Stride<TI, 13, 4, 1>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2, 6>;
        using STRIDE = rlt::tensor::Stride<TI, 13, 4, 1>;
        bool grm = test_row_major<SHAPE, STRIDE>() == false;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2, 1>;
        using STRIDE = rlt::tensor::Stride<TI, 7, 4, 1>;
        bool grm = test_row_major<SHAPE, STRIDE>() == false;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2>;
        using STRIDE = rlt::tensor::Stride<TI, 7>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2>;
        using STRIDE = rlt::tensor::Stride<TI, 7, 2>;
        bool grm = test_row_major<SHAPE, STRIDE>() == true;
        ASSERT_TRUE(grm);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 1>;
        using STRIDE = rlt::tensor::Stride<TI, 1, 2>;
        bool grm = test_row_major<SHAPE, STRIDE>() == false;
        ASSERT_TRUE(grm);
    }


}

TEST(RL_TOOLS_TENSOR_TEST, STRIDE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    std::cout << "dim[0]: " << rlt::get<0>(SHAPE{}) << " stride[0]: " << rlt::get<0>(STRIDE{}) << std::endl;
    std::cout << "dim[1]: " << rlt::get<1>(SHAPE{}) << " stride[1]: " << rlt::get<1>(STRIDE{}) << std::endl;
    std::cout << "dim[2]: " << rlt::get<2>(SHAPE{}) << " stride[2]: " << rlt::get<2>(STRIDE{}) << std::endl;
}

TEST(RL_TOOLS_TENSOR_TEST, MALLOC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    DEVICE device;
    rlt::malloc(device, tensor);
}

TEST(RL_TOOLS_TENSOR_TEST, SET){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    DEVICE device;
    rlt::malloc(device, tensor);
    rlt::set(device, tensor, 1337, 0, 0, 0);
    rlt::set(device, tensor, 1337, 1, 2, 0);
    rlt::print(device, tensor);
}

TEST(RL_TOOLS_TENSOR_TEST, VIEW){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    DEVICE device;
    rlt::malloc(device, tensor);
    rlt::set(device, tensor, 1337, 0, 0, 0);
    rlt::set(device, tensor, 1337, 1, 2, 0);
    rlt::print(device, tensor);
    {
        auto view = rlt::view(device, tensor, 1);
        rlt::print(device, view);
        rlt::set(device, view, 1338, 2, 0);
        T value = rlt::get(device, tensor, 1, 2, 0);
        ASSERT_EQ(value, 1338);
    }

    {
        constexpr TI DIM = 1;
        for (TI i = 0; i < rlt::get<DIM>(SHAPE{}); i++){
            auto view2 = rlt::view(device, tensor, i, rlt::tensor::ViewSpec<DIM>{});
            ASSERT_EQ(rlt::data(tensor) + i * rlt::get<DIM>(STRIDE{}), rlt::data(view2));
            rlt::set(device, view2, (T)(DIM * i), 1, 0);
            ASSERT_EQ(rlt::get(device, tensor, 1, i, 0), (T)(DIM*i));
        }
    }

    {
        constexpr TI DIM = 2;
        for (TI i = 0; i < rlt::get<DIM>(SHAPE{}); i++){
            auto view2 = rlt::view(device, tensor, i, rlt::tensor::ViewSpec<DIM>{});
            ASSERT_EQ(rlt::data(tensor) + i * rlt::get<DIM>(STRIDE{}), rlt::data(view2));
            rlt::set(device, view2, (T)(DIM * i), 1, 0);
            ASSERT_EQ(rlt::get(device, tensor, 1, 0, i), (T)(DIM*i));
        }
    }
    std::cout << "afterwards: " << std::endl;
    rlt::print(device, tensor);
}

TEST(RL_TOOLS_TENSOR_TEST, RANDN){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        rlt::print(device, tensor);
        rlt::free(device, tensor);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
//        rlt::print(device, tensor);
        T sum = rlt::sum(device, tensor);
        std::cout << "sum: " << sum << std::endl;
        T num_elements = rlt::get<0>(rlt::tensor::CumulativeProduct<SHAPE>{});
        ASSERT_EQ(num_elements, 20*30*40);
        ASSERT_LT(rlt::math::abs(device.math, sum), rlt::math::sqrt(device.math, num_elements) * 5);
        rlt::free(device, tensor);
    }
}

template<typename DEVICE, typename SPEC>
typename SPEC::T sum_old(DEVICE& device, rlt::Tensor<SPEC>& t){
    using T = typename SPEC::T;
    using TI = typename DEVICE::index_t;
    if constexpr(rlt::length(typename SPEC::SHAPE{}) > 1){
        T acc = 0;
        for(TI i=0; i < rlt::get<0>(typename SPEC::SHAPE{}); ++i){
            auto next = rlt::view(device, t, i);
            acc += sum_old(device, next);
        }
        return acc;
    }
    else{
        T acc = 0;
        for(TI i=0; i < rlt::get<0>(typename SPEC::SHAPE{}); i++){
            acc += rlt::get(device, t, i);
        }
        return acc;
    }
}

TEST(RL_TOOLS_TENSOR_TEST, SUM){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 2>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::set(device, tensor, 1, 0);
        rlt::set(device, tensor, 2, 1);
        T sum = rlt::sum(device, tensor);
        ASSERT_EQ(sum, 1+2);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::set(device, tensor, 1, 0, 0);
        rlt::set(device, tensor, 2, 0, 1);
        rlt::set(device, tensor, 3, 0, 2);
        rlt::set(device, tensor, 4, 1, 0);
        rlt::set(device, tensor, 5, 1, 1);
        rlt::set(device, tensor, 6, 1, 2);
        T sum = rlt::sum(device, tensor);
        ASSERT_EQ(sum, 1+2+3+4+5+6);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 4>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::set_all(device, tensor, 1337);
        T sum = rlt::sum(device, tensor);
        ASSERT_EQ(sum, 1337 * rlt::get<0>(rlt::tensor::CumulativeProduct<SHAPE>{}));
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 3, 4>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        T sum = rlt::sum(device, tensor);
        T sum2 = sum_old(device, tensor);
        ASSERT_NEAR(sum, sum2, EPSILON);
    }
#ifdef NDEBUG
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 10, 10, 10, 10, 10, 1>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        constexpr TI NUM_ITERATIONS = 100;
        T sum, sum2;
        {
            auto start = std::chrono::high_resolution_clock::now();
            for(TI iteration_i=0; iteration_i < NUM_ITERATIONS; iteration_i++){
                sum2 = sum_old(device, tensor);
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "time for sum_old: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        {
            auto start = std::chrono::high_resolution_clock::now();
            for(TI iteration_i=0; iteration_i < NUM_ITERATIONS; iteration_i++){
                sum = rlt::sum(device, tensor);
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "time for rlt::sum: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        }
        std::cout << "sum: " << sum << " sum2: " << sum2 << std::endl;
        ASSERT_NEAR(sum, sum2, EPSILON);
    }
#endif
}
TEST(RL_TOOLS_TENSOR_TEST, COMPARE_DIMS) {
    using DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;
    {
        using SHAPE1 = rlt::tensor::Shape<TI, 20, 30, 40>;
        using SHAPE2 = rlt::tensor::Shape<TI, 20, 30, 40>;
        using SHAPE3 = rlt::tensor::Shape<TI, 10, 31, 41>;
        static_assert(rlt::length(rlt::tensor::PopFront<rlt::tensor::PopFront<rlt::tensor::PopFront<SHAPE1>>>{}) == 0);
        static_assert(rlt::utils::typing::is_same_v<SHAPE1::NEXT_ELEMENT::NEXT_ELEMENT::NEXT_ELEMENT::NEXT_ELEMENT, rlt::tensor::FinalElement>);
        static_assert(rlt::tensor::same_dimensions_shape<SHAPE1, SHAPE2>());
        static_assert(!rlt::tensor::same_dimensions_shape<SHAPE1, SHAPE3>());
    }
}
TEST(RL_TOOLS_TENSOR_TEST, SET_ALL) {
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    rlt::malloc(device, tensor);
    rlt::randn(device, tensor, rng);
    rlt::set_all(device, tensor, 1337);
    for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
        for(TI dim1=0; dim1 < rlt::get<1>(SHAPE{}); dim1++){
            for(TI dim2=0; dim2 < rlt::get<2>(SHAPE{}); dim2++){
                ASSERT_EQ(rlt::get(device, tensor, dim0, dim1, dim2), 1337);
            }
        }
    }
}
TEST(RL_TOOLS_TENSOR_TEST, SUBTRACT) {
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor, tensor2, diff, abs_diff;
    DEVICE device;
    rlt::malloc(device, tensor);
    rlt::malloc(device, tensor2);
    rlt::malloc(device, diff);
    rlt::malloc(device, abs_diff);
    rlt::set_all(device, tensor, 1337);
    rlt::set_all(device, tensor2, 1338);
    rlt::subtract(device, tensor, tensor2, diff);
    T sum = rlt::sum(device, diff);
    ASSERT_EQ(sum, -(T)rlt::get<0>(rlt::tensor::CumulativeProduct<SHAPE>{}));
    rlt::copy(device, device, diff, abs_diff);
    rlt::abs(device, abs_diff);
    T sum_abs = rlt::sum(device, abs_diff);
    ASSERT_EQ(sum_abs, rlt::get<0>(rlt::tensor::CumulativeProduct<SHAPE>{}));
}
TEST(RL_TOOLS_TENSOR_TEST, COPY){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor, tensor_target, tensor_target2, diff;
        rlt::malloc(device, tensor);
        rlt::malloc(device, tensor_target);
        rlt::malloc(device, tensor_target2);
        rlt::malloc(device, diff);
        rlt::randn(device, tensor, rng);

        for(TI i=0; i < rlt::get<1>(SHAPE{})/2; i++){
            auto view = rlt::view(device, tensor, i, rlt::tensor::ViewSpec<1>{});
            static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
            static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
            auto view_target = rlt::view(device, tensor_target, rlt::get<1>(SHAPE{})/2 + i, rlt::tensor::ViewSpec<1>{});
            rlt::copy(device, device, view, view_target);
        }

        for(TI i=rlt::get<1>(SHAPE{})/2; i < rlt::get<1>(SHAPE{}); i++){
            auto view = rlt::view(device, tensor, i, rlt::tensor::ViewSpec<1>{});
            static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
            static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
            auto view_target = rlt::view(device, tensor_target, i-rlt::get<1>(SHAPE{})/2, rlt::tensor::ViewSpec<1>{});
            rlt::copy(device, device, view, view_target);
        }

        for(TI i=0; i < rlt::get<1>(SHAPE{})/2; i++){
            auto view = rlt::view(device, tensor_target, i, rlt::tensor::ViewSpec<1>{});
            static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
            static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
            auto view_target = rlt::view(device, tensor_target2, rlt::get<1>(SHAPE{})/2 + i, rlt::tensor::ViewSpec<1>{});
            rlt::copy(device, device, view, view_target);
        }

        for(TI i=rlt::get<1>(SHAPE{})/2; i < rlt::get<1>(SHAPE{}); i++){
            auto view = rlt::view(device, tensor_target, i, rlt::tensor::ViewSpec<1>{});
            static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
            static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
            auto view_target = rlt::view(device, tensor_target2, i-rlt::get<1>(SHAPE{})/2, rlt::tensor::ViewSpec<1>{});
            rlt::copy(device, device, view, view_target);
        }

        rlt::subtract(device, tensor, tensor_target2, diff);
        rlt::abs(device, diff);
        T abs_diff = rlt::sum(device, diff);

        std::cout << "Abs diff: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, EPSILON);


        rlt::free(device, tensor);
        rlt::free(device, tensor_target);
    }
}

TEST(RL_TOOLS_TENSOR_TEST, VIEW_RANGE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 10, rlt::tensor::ViewSpec<1, 11>{});
        static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        rlt::set_all(device, view, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            for(TI dim1=0; dim1 < rlt::get<1>(SHAPE{}); dim1++){
                for(TI dim2=0; dim2 < rlt::get<2>(SHAPE{}); dim2++){
                    if(dim1 >= 10 && dim1 < 21){
                        ASSERT_EQ(rlt::get(device, tensor, dim0, dim1, dim2), 1337);
                    }
                }
            }
        }
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 3, rlt::tensor::ViewSpec<0, 11>{});
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>()); // should be dense because the range is in DIM=0
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        rlt::set_all(device, view, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            for(TI dim1=0; dim1 < rlt::get<1>(SHAPE{}); dim1++){
                for(TI dim2=0; dim2 < rlt::get<2>(SHAPE{}); dim2++){
                    if(dim0 >= 3 && dim0 < 14){
                        ASSERT_EQ(rlt::get(device, tensor, dim0, dim1, dim2), 1337);
                    }
                }
            }
        }
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 29, rlt::tensor::ViewSpec<2, 11>{});
        static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        rlt::set_all(device, view, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            for(TI dim1=0; dim1 < rlt::get<1>(SHAPE{}); dim1++){
                for(TI dim2=0; dim2 < rlt::get<2>(SHAPE{}); dim2++){
                    if(dim2 >= 29 && dim2 < 40){
                        ASSERT_EQ(rlt::get(device, tensor, dim0, dim1, dim2), 1337);
                    }
                }
            }
        }
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 1, rlt::tensor::ViewSpec<0, 11>{});
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());// should be dense because the range is in DIM=0
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        rlt::set_all(device, view, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            if(dim0 >= 1 && dim0 < 12){
                ASSERT_EQ(rlt::get(device, tensor, dim0), 1337);
            }
        }
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 0, rlt::tensor::ViewSpec<0, 11>{});
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>()); // should be dense because the range is in DIM=0
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        rlt::set_all(device, view, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            if(dim0 < 11){
                ASSERT_EQ(rlt::get(device, tensor, dim0), 1337);
            }
        }
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 20, 30, 40>;
        using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        auto view = rlt::view_range(device, tensor, 29, rlt::tensor::ViewSpec<2, 11>{});
        static_assert(!rlt::tensor::dense_row_major_layout<typename decltype(view)::SPEC>());
        static_assert(rlt::tensor::dense_row_major_layout<typename decltype(tensor)::SPEC>());
        auto view2 = rlt::view_range(device, view, 3, rlt::tensor::ViewSpec<0, 5>{});
        rlt::set_all(device, view2, 1337);
        for(TI dim0=0; dim0 < rlt::get<0>(SHAPE{}); dim0++){
            for(TI dim1=0; dim1 < rlt::get<1>(SHAPE{}); dim1++){
                for(TI dim2=0; dim2 < rlt::get<2>(SHAPE{}); dim2++){
                    if(dim2 >= 29 && dim2 < 34){
                        if(dim0 >= 3 && dim0 < 8){
                            ASSERT_EQ(rlt::get(device, tensor, dim0, dim1, dim2), 1337);
                        }
                    }
                }
            }
        }
    }
}

TEST(RL_TOOLS_TENSOR_TEST, ABSOLUTE_DIFFERENCE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TI = DEVICE::index_t;
    DEVICE device;
    {
        rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 2, 2>>> A, B;
        rlt::malloc(device, A);
        rlt::malloc(device, B);
        set(device, A, 1, 0, 0);
        set(device, A, 2, 0, 1);
        set(device, A, 3, 1, 0);
        set(device, A, 4, 1, 1);
        set(device, B, 2, 0, 0);
        set(device, B, 1, 0, 1);
        set(device, B, 4, 1, 0);
        set(device, B, 3, 1, 1);

        T diff = rlt::abs_diff(device, A, B);
        ASSERT_EQ(diff, 4);
    }
    {
        rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 2>>> A, B;
        rlt::malloc(device, A);
        rlt::malloc(device, B);
        set(device, A, 1, 0);
        set(device, A, 2, 1);
        set(device, B, 2, 0);
        set(device, B, 2, 1);
        T diff = rlt::abs_diff(device, A, B);
        ASSERT_EQ(diff, 1);
    }
}

TEST(RL_TOOLS_TENSOR_TEST, MATRIX_MULTIPLICATION_GENERIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 2, 2>>> A, B, C, C_target;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    set(device, A, -0.259093, 0, 0);
    set(device, A, -1.498961, 0, 1);
    set(device, A, +0.119264, 1, 0);
    set(device, A, +0.458181, 1, 1);

    set(device, B, +0.394975, 0, 0);
    set(device, B, +0.044197, 0, 1);
    set(device, B, -0.636256, 1, 0);
    set(device, B, +1.731264, 1, 1);

    set(device, C_target, -0.259093 * +0.394975 + -1.498961 * -0.636256, 0, 0);
    set(device, C_target, -0.259093 * +0.044197 + -1.498961 * +1.731264, 0, 1);
    set(device, C_target, +0.119264 * +0.394975 + +0.458181 * -0.636256, 1, 0);
    set(device, C_target, +0.119264 * +0.044197 + +0.458181 * +1.731264, 1, 1);
    rlt::print(device, C_target);

    rlt::matrix_multiply(device, A, B, C);
    rlt::print(device, C);
    auto diff = rlt::abs_diff(device, C_target, C);
    std::cout << "Matrix mul diff: " << diff << std::endl;
    ASSERT_TRUE(diff < EPSILON);
}



template <typename SHAPE>
bool test_element_wise_multiply_accumulate(){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A, B, C;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A_target, B_target, C_target;
        rlt::malloc(device, A);
        rlt::malloc(device, B);
        rlt::malloc(device, C);
        rlt::malloc(device, A_target);
        rlt::malloc(device, B_target);
        rlt::malloc(device, C_target);
        rlt::randn(device, A, rng);
        rlt::randn(device, B, rng);
        rlt::randn(device, C, rng);
        T manual = rlt::get_flat(device, A, 0) * rlt::get_flat(device, B, 0) + rlt::get_flat(device, C, 0);
        rlt::copy(device, device, A, A_target);
        rlt::copy(device, device, B, B_target);
        rlt::copy(device, device, C, C_target);
        T manual_target = rlt::get_flat(device, A_target, 0) * rlt::get_flat(device, B, 0) + rlt::get_flat(device, C_target, 0);
        rlt::multiply_accumulate(device, A, B, C);
        rlt::multiply(device, B_target, A_target);
        rlt::add(device, A_target, C_target);
        std::cout << "manual: " << manual << std::endl;
        std::cout << "manual_target: " << manual_target << std::endl;
        std::cout << "C:" << std::endl;
        rlt::print(device, C);
        std::cout << "C_target:" << std::endl;
        rlt::print(device, C_target);
        T diff = rlt::abs_diff(device, C_target, C);
        std::cout << "Element wise multiply accumulate diff: " << diff << std::endl;
        bool good = manual == manual_target;
        good &= diff < EPSILON;
        good &= get_flat(device, C, 0) == manual;
        return good;
    }
}

TEST(RL_TOOLS_TENSOR_TEST, ELEMENT_WISE_MULTIPLY_ACCUMULATE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A, B, C, C_target;
        rlt::malloc(device, A);
        rlt::malloc(device, B);
        rlt::malloc(device, C);
        rlt::malloc(device, C_target);
        rlt::set(device, A, 10, 0, 0);
        rlt::set(device, A, 2, 0, 1);
        rlt::set(device, A, 3, 1, 0);
        rlt::set(device, A, 4, 1, 1);
        rlt::set(device, B, 2, 0, 0);
        rlt::set(device, B, 1, 0, 1);
        rlt::set(device, B, 4, 1, 0);
        rlt::set(device, B, 3, 1, 1);
        rlt::set_all(device, C, 0);
        rlt::set(device, C, 1337, 1, 0);
        rlt::multiply_accumulate(device, A, B, C);
        rlt::set(device, C_target, 20, 0, 0);
        rlt::set(device, C_target, 2, 0, 1);
        rlt::set(device, C_target, 12+1337, 1, 0);
        rlt::set(device, C_target, 12, 1, 1);
        rlt::print(device, C);
        T diff = rlt::abs_diff(device, C_target, C);
        std::cout << "Element wise multiply accumulate diff: " << diff << std::endl;
        ASSERT_TRUE(diff < EPSILON);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 1>;
        bool good = test_element_wise_multiply_accumulate<SHAPE>();
        ASSERT_TRUE(good);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10>;
        bool good = test_element_wise_multiply_accumulate<SHAPE>();
        ASSERT_TRUE(good);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 1>;
        bool good = test_element_wise_multiply_accumulate<SHAPE>();
        ASSERT_TRUE(good);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 1, 1, 1, 1, 1>;
        bool good = test_element_wise_multiply_accumulate<SHAPE>();
        ASSERT_TRUE(good);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 1, 1, 1, 1, 10>;
        bool good = test_element_wise_multiply_accumulate<SHAPE>();
        ASSERT_TRUE(good);
    }
}

TEST(RL_TOOLS_TENSOR_TEST, PERMUTE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A;
        rlt::malloc(device, A);
        rlt::set(device, A, 1, 0, 0);
        rlt::set(device, A, 2, 0, 1);
        rlt::set(device, A, 3, 1, 0);
        rlt::set(device, A, 4, 1, 1);
        auto permuted = rlt::permute(device, A, rlt::tensor::PermutationSpec<1, 0>{});
        static_assert(decltype(A)::SPEC::SIZE == decltype(permuted)::SPEC::SIZE);
        std::cout << "Old stride: " << rlt::get<0>(typename decltype(A)::SPEC::STRIDE{}) << " " << rlt::get<1>(typename decltype(A)::SPEC::STRIDE{}) << std::endl;
        std::cout << "A: " << std::endl;
        rlt::print(device, A);
        std::cout << "New stride: " << rlt::get<0>(typename decltype(permuted)::SPEC::STRIDE{}) << " " << rlt::get<1>(typename decltype(permuted)::SPEC::STRIDE{}) << std::endl;
        std::cout << "A permuted: " << std::endl;
        rlt::print(device, permuted);
        ASSERT_EQ(rlt::get(device, permuted, 0, 0), 1);
        ASSERT_EQ(rlt::get(device, permuted, 1, 0), 2);
        ASSERT_EQ(rlt::get(device, permuted, 0, 1), 3);
        ASSERT_EQ(rlt::get(device, permuted, 1, 1), 4);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 3, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A;
        rlt::malloc(device, A);
        rlt::set(device, A, 1, 0, 0);
        rlt::set(device, A, 2, 0, 1);
        rlt::set(device, A, 3, 1, 0);
        rlt::set(device, A, 4, 1, 1);
        rlt::set(device, A, 5, 2, 0);
        rlt::set(device, A, 6, 2, 1);
        auto permuted = rlt::permute(device, A, rlt::tensor::PermutationSpec<1, 0>{});
        std::cout << "Old stride: " << rlt::get<0>(typename decltype(A)::SPEC::STRIDE{}) << " " << rlt::get<1>(typename decltype(A)::SPEC::STRIDE{}) << std::endl;
        std::cout << "A: " << std::endl;
        rlt::print(device, A);
        std::cout << "New stride: " << rlt::get<0>(typename decltype(permuted)::SPEC::STRIDE{}) << " " << rlt::get<1>(typename decltype(permuted)::SPEC::STRIDE{}) << std::endl;
        std::cout << "A permuted: " << std::endl;
        rlt::print(device, permuted);
        ASSERT_EQ(rlt::get(device, permuted, 0, 0), 1);
        ASSERT_EQ(rlt::get(device, permuted, 1, 0), 2);
        ASSERT_EQ(rlt::get(device, permuted, 0, 1), 3);
        ASSERT_EQ(rlt::get(device, permuted, 1, 1), 4);
        ASSERT_EQ(rlt::get(device, permuted, 0, 2), 5);
        ASSERT_EQ(rlt::get(device, permuted, 1, 2), 6);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 3, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A;
        rlt::malloc(device, A);
        rlt::randn(device, A, rng);
        auto permuted = rlt::permute(device, A, rlt::tensor::PermutationSpec<1, 0>{});
        for(TI i=0; i < rlt::get<0>(SHAPE{}); i++){
            for(TI j=0; j < rlt::get<1>(SHAPE{}); j++){
                for(TI k=0; k < rlt::get<2>(SHAPE{}); k++){
                    ASSERT_EQ(rlt::get(device, A, i, j, k), rlt::get(device, permuted, j, i, k));
                }
            }
        }
    }
}

TEST(RL_TOOLS_TENSOR_TEST, REDUCE_ALONG_DIMENSION){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 2>;
        using SHAPE_OUTPUT = rlt::tensor::Shape<TI, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> input;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE_OUTPUT>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::set(device, input, 1, 0, 0);
        rlt::set(device, input, 2, 0, 1);
        rlt::set(device, input, 3, 1, 0);
        rlt::set(device, input, 4, 1, 1);
        rlt::reduce_sum(device, input, output);
        ASSERT_EQ(rlt::get(device, output, 0), 1+2);
        ASSERT_EQ(rlt::get(device, output, 1), 3+4);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 2, 1>;
        using SHAPE_OUTPUT = rlt::tensor::Shape<TI, 2>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> input;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE_OUTPUT>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::set(device, input, 1, 0, 0);
        rlt::set(device, input, 3, 1, 0);
        rlt::reduce_sum(device, input, output);
        ASSERT_EQ(rlt::get(device, output, 0), 1);
        ASSERT_EQ(rlt::get(device, output, 1), 3);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 1, 1>;
        using SHAPE_OUTPUT = rlt::tensor::Shape<TI, 1>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> input;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE_OUTPUT>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::set(device, input, 1337, 0, 0);
        rlt::reduce_sum(device, input, output);
        ASSERT_EQ(rlt::get(device, output, 0), 1337);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 1, 1, 1, 1, 100>;
        using SHAPE_OUTPUT = rlt::tensor::Shape<TI, 1, 1, 1, 1>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> input;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE_OUTPUT>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::set_all(device, input, 0);
        rlt::set(device, input, 1337, 0, 0, 0, 0, 50);
        rlt::set(device, input, -1337, 0, 0, 0, 0, 0);
        rlt::set(device, input, 1338, 0, 0, 0, 0, 99);
        rlt::reduce_sum(device, input, output);
        ASSERT_EQ(rlt::get(device, output, 0, 0, 0, 0), 1338);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 1, 1, 6, 1, 100>;
        using SHAPE_OUTPUT = rlt::tensor::Shape<TI, 1, 1, 6, 1>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> input;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE_OUTPUT>> output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);
        rlt::set_all(device, input, 0);
        rlt::set(device, input, 1337, 0, 0, 0, 0, 50);
        rlt::set(device, input, -1337, 0, 0, 0, 0, 0);
        rlt::set(device, input, 1338, 0, 0, 0, 0, 99);
        rlt::set(device, input, 1, 0, 0, 1, 0, 5);
        rlt::set(device, input, 2, 0, 0, 2, 0, 0);
        rlt::set(device, input, 3, 0, 0, 3, 0, 80);
        rlt::set(device, input, 4, 0, 0, 4, 0, 37);
        rlt::set(device, input, 5, 0, 0, 5, 0, 99);
        rlt::reduce_sum(device, input, output);
        ASSERT_EQ(rlt::get(device, output, 0, 0, 0, 0), 1338);
        for(TI i=1; i < 6; i++){
            ASSERT_EQ(rlt::get(device, output, 0, 0, i, 0), i);
        }
    }
}

TEST(RL_TOOLS_TENSOR_TEST, RESHAPE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using SHAPE = rlt::tensor::Shape<TI, 10>;
    using STRIDE = rlt::tensor::Stride<TI, 3>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE>> tensor;
    rlt::malloc(device, tensor);
    rlt::randn(device, tensor, rng);
    auto reshaped = rlt::reshape_row_major(device, tensor, rlt::tensor::Shape<TI, 5, 2>{});
    static_assert(rlt::get<0>(decltype(reshaped)::SPEC::STRIDE{}) == 6);
    static_assert(rlt::get<1>(decltype(reshaped)::SPEC::STRIDE{}) == 3);
    for(TI i=0; i < 5; i++){
        for(TI j=0; j < 2; j++){
            ASSERT_EQ(rlt::get(device, tensor, i*2+j), rlt::get(device, reshaped, i, j));
        }
    }
    rlt::print(device, tensor);
    rlt::print(device, reshaped);
}

TEST(RL_TOOLS_TENSOR_TEST, IS_NAN){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 10>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        bool is_nan = rlt::is_nan(device, tensor);
        ASSERT_FALSE(is_nan);
        rlt::set(device, tensor, rlt::math::nan<T>(device.math), 3);
        bool is_nan_now = rlt::is_nan(device, tensor);
        ASSERT_TRUE(is_nan_now);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 5>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        bool is_nan = rlt::is_nan(device, tensor);
        ASSERT_FALSE(is_nan);
        rlt::set(device, tensor, rlt::math::nan<T>(device.math), 3, 2);
        rlt::print(device, tensor);
        bool is_nan_now = rlt::is_nan(device, tensor);
        ASSERT_TRUE(is_nan_now);
    }
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 5, 3>;
        rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> tensor;
        rlt::malloc(device, tensor);
        rlt::randn(device, tensor, rng);
        bool is_nan = rlt::is_nan(device, tensor);
        ASSERT_FALSE(is_nan);
        rlt::set(device, tensor, rlt::math::nan<T>(device.math), 3, 2, 2);
        rlt::print(device, tensor);
        bool is_nan_now = rlt::is_nan(device, tensor);
        ASSERT_TRUE(is_nan_now);
    }
}

TEST(RL_TOOLS_TENSOR_TEST, SHAPE_GETTER){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    {
        using SHAPE = rlt::tensor::Shape<TI, 10, 5, 3>;
        ASSERT_EQ(SHAPE::VALUE, 10);
        ASSERT_EQ(SHAPE::NEXT_ELEMENT::VALUE, 5);
        ASSERT_EQ(SHAPE::NEXT_ELEMENT::NEXT_ELEMENT::VALUE, 3);
        ASSERT_EQ(SHAPE::NEXT_ELEMENT::NEXT_ELEMENT::NEXT_ELEMENT::VALUE, 0);
        ASSERT_EQ(SHAPE::GET<0>, 10);
        ASSERT_EQ(SHAPE::GET<1>, 5);
        ASSERT_EQ(SHAPE::GET<2>, 3);
        ASSERT_EQ(SHAPE::LENGTH, 3);
        ASSERT_EQ(SHAPE::FIRST, 10);
        ASSERT_EQ(SHAPE::LAST, 3);
    }
}

TEST(RL_TOOLS_TENSOR_TEST, SQUARED_SUM){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);
    using SHAPE = rlt::tensor::Shape<TI, 10, 5, 3>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, false>> tensor;
    rlt::randn(device, tensor, rng);
    T squared_sum = rlt::squared_sum(device, tensor);
    T manual = 0;
    for(TI i=0; i < 10; i++){
        for(TI j=0; j < 5; j++){
            for(TI k=0; k < 3; k++){
                manual += rlt::get(device, tensor, i, j, k) * rlt::get(device, tensor, i, j, k);
            }
        }
    }
    std::cout << "Squared sum: " << squared_sum << std::endl;
    std::cout << "Manual: " << manual << std::endl;
    ASSERT_EQ(squared_sum, manual);

    auto matrix_view = rlt::matrix_view(device, tensor);
    T squared_sum_matrix = rlt::squared_sum(device, matrix_view);
    std::cout << "Squared sum matrix: " << squared_sum_matrix << std::endl;
    ASSERT_EQ(squared_sum_matrix, squared_sum);
}
