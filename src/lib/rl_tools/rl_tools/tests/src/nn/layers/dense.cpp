
#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
using DEVICE = rlt::devices::DefaultCPU;
using T = float;
using TI = typename DEVICE::index_t;
DEVICE device;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
TI seed = 1;
DEVICE::SPEC::RANDOM::ENGINE<> rng;

constexpr TI INPUT_DIM = 5;
constexpr TI OUTPUT_DIM = 5;
constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::RELU;
using PARAMETER_TYPE = rlt::nn::parameters::Plain;

using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, 1, INPUT_DIM>;
using LAYER_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, ACTIVATION_FUNCTION>;

#include <chrono>

#include <gtest/gtest.h>
#include <cstring>



TEST(RL_TOOLS_NN_LAYERS_DENSE, COPY_REGRESSION) {
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);

    rlt::nn::layers::dense::Layer<LAYER_CONFIG, rlt::nn::capability::Forward<>, INPUT_SHAPE> layer;
    decltype(layer)::template Buffer<1> buffer;
    rlt::malloc(device, layer);
    rlt::malloc(device, buffer);
    rlt::init_weights(device, layer, rng);
    constexpr TI BATCH_SIZE = 1;
    rlt::Matrix<rlt::matrix::Specification<T, TI, BATCH_SIZE, INPUT_DIM>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, BATCH_SIZE, INPUT_DIM>> output;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::randn(device, input, rng);
    rlt::print(device, input);
    rlt::evaluate(device, layer, input, output, buffer, rng);
    using PARAMETER_TYPE_2 = rlt::nn::parameters::Gradient;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, ACTIVATION_FUNCTION>;
    rlt::nn::layers::dense::Layer<LAYER_2_CONFIG, rlt::nn::capability::Gradient<rlt::nn::parameters::Gradient>, INPUT_SHAPE> layer_2;
    rlt::malloc(device, layer_2);
    rlt::copy(device, device, layer, layer_2);
    rlt::zero_gradient(device, layer_2);
    auto abs_diff = rlt::abs_diff(device, layer, layer_2);
    EXPECT_EQ(abs_diff, 0);
}

TEST(RL_TOOLS_NN_LAYERS_DENSE, COPY_TIMING) {
    rlt::init(device);
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 100, 100>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 100, 100>> output;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    constexpr TI ITERATIONS = 1000;
    {
        auto start = std::chrono::high_resolution_clock::now();
        for(TI i = 0; i < ITERATIONS; i++){
            std::memcpy(output._data, input._data, decltype(input)::SPEC::SIZE_BYTES);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "memcpy: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    }
    {
        auto start = std::chrono::high_resolution_clock::now();
        for(TI i = 0; i < ITERATIONS; i++){
            for(TI i = 0; i < decltype(input)::SPEC::SIZE; i++){
                output._data[i] = input._data[i];
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "memcpy: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    }
    rlt::free(device, input);
    rlt::free(device, output);
}

