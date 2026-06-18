#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/layers/dense/operations_cpu_mkl.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DEVICE_MKL = rlt::devices::DEVICE_FACTORY<>;
using DEVICE_GENERIC = rlt::devices::DefaultCPU;



#include <gtest/gtest.h>
#include <cstring>


template <typename TYPE_POLICY, typename TI, TI INPUT_DIM, TI OUTPUT_DIM, TI BATCH_SIZE>
void test(){
    using T = typename TYPE_POLICY::DEFAULT;
    DEVICE_MKL device_mkl;
    DEVICE_GENERIC device_generic;
    TI seed = 1;
    DEVICE_MKL::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device_mkl, rng);
    rlt::init(device_mkl, rng, seed);

//    constexpr TI INPUT_DIM = 5;
//    constexpr TI OUTPUT_DIM = 5;
    constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::RELU;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
    using LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, ACTIVATION_FUNCTION>;

    rlt::nn::layers::dense::Layer<LAYER_CONFIG, rlt::nn::capability::Forward<>, INPUT_SHAPE> layer;
    typename decltype(layer)::template Buffer<> layer_buffer;
    rlt::malloc(device_generic, layer);
    rlt::malloc(device_generic, layer_buffer);
    rlt::init_weights(device_generic, layer, rng);
//    constexpr TI BATCH_SIZE = 1;
    rlt::Matrix<rlt::matrix::Specification<T, TI, BATCH_SIZE, INPUT_DIM>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, BATCH_SIZE, OUTPUT_DIM>> output_generic, output_mkl;
    rlt::malloc(device_generic, input);
    rlt::malloc(device_generic, output_generic);
    rlt::malloc(device_generic, output_mkl);
    rlt::randn(device_generic, input, rng);
    rlt::print(device_generic, input);
    rlt::evaluate(device_generic, layer, input, output_generic, layer_buffer, rng);
    rlt::evaluate(device_mkl, layer, input, output_mkl, layer_buffer, rng);
    auto diff = rlt::abs_diff(device_generic, output_generic, output_mkl);
    T diff_per_element = diff / (BATCH_SIZE * OUTPUT_DIM);
    std::cout << "Matrix mul diff: " << diff << " per element: " << diff_per_element << std::endl;
    if(rlt::utils::typing::is_same_v<T, float>){
        ASSERT_TRUE(diff_per_element < 1e-5);
    }else{
        ASSERT_TRUE(diff_per_element < 1e-10);
    }
}

TEST(RL_TOOLS_NN_LAYERS_DENSE, COPY_REGRESSION_MKL) {
    using TI = typename DEVICE_MKL::index_t;
    test<rlt::numeric_types::Policy<float>, TI, 5, 5, 1>();
    test<rlt::numeric_types::Policy<float>, TI, 5, 5, 2>();
    test<rlt::numeric_types::Policy<float>, TI, 2, 5, 10>();
    test<rlt::numeric_types::Policy<float>, TI, 3, 5, 100>();
    test<rlt::numeric_types::Policy<float>, TI, 15, 16, 80>();
    test<rlt::numeric_types::Policy<float>, TI, 15, 16, 81>();
}
