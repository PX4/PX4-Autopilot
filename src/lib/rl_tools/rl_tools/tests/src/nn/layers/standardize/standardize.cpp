#include <rl_tools/operations/cpu.h>

#include <gtest/gtest.h>
#include <rl_tools/nn/parameters/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

TEST(RL_TOOLS_NN_LAYERS_STANDARDIZE, FORWARD_DEFAULT){
    constexpr TI DIM = 10;
    using CONFIG = rlt::nn::layers::standardize::Configuration<TYPE_POLICY, TI>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    constexpr TI BATCH_SIZE = 1;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, DIM>;
    rlt::nn::layers::standardize::Layer<CONFIG, CAPABILITY, INPUT_SHAPE> layer;
    typename decltype(layer)::template Buffer<BATCH_SIZE> buffer;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, DIM, false>> input, output;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::malloc(device, layer);
    rlt::malloc(device, buffer);
    rlt::init_weights(device, layer, rng);
    rlt::randn(device, input, rng);
    rlt::print(device, input);
    rlt::evaluate(device, layer, input, output, buffer, rng);
    T diff = rlt::abs_diff(device, input, output);
    ASSERT_LT(diff, 1e-10);

}

TEST(RL_TOOLS_NN_LAYERS_STANDARDIZE, FORWARD){
    constexpr TI DIM = 10;
    constexpr TI BATCH_SIZE = 1000000;
    using CONFIG = rlt::nn::layers::standardize::Configuration<TYPE_POLICY, TI>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, DIM>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    rlt::nn::layers::standardize::Layer<CONFIG, CAPABILITY, INPUT_SHAPE> layer;
    typename decltype(layer)::template Buffer<> buffer;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, DIM>> mean, std, bias, variance;
    rlt::Matrix<rlt::matrix::Specification<T, TI, BATCH_SIZE, DIM>> input, output;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::malloc(device, layer);
    rlt::malloc(device, buffer);
    rlt::malloc(device, mean);
    rlt::malloc(device, std);
    rlt::malloc(device, bias);
    rlt::malloc(device, variance);
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::init_weights(device, layer, rng);
    rlt::randn(device, input, rng);

    for(TI dim_i=0; dim_i < DIM; dim_i++){
        T bias_value = rlt::random::normal_distribution::sample(device.random, (T)0, (T)10, rng);
        T variance_value = rlt::random::normal_distribution::sample(device.random, (T)0.01, (T)10, rng);
        rlt::set(bias, 0, dim_i, bias_value);
        rlt::set(variance, 0, dim_i, rlt::math::abs(device.math, variance_value));
        for(TI input_i=0; input_i < BATCH_SIZE; input_i++){
            T value = rlt::get(input, input_i, dim_i);
            rlt::set(input, input_i, dim_i, bias_value + value * variance_value);
        }
    }
    rlt::mean_std_colwise(device, input, mean, std);
    rlt::set_statistics(device, layer, mean, std);
    rlt::evaluate(device, layer, input, output, buffer, rng);
    T output_mean = rlt::mean(device, output);
    T output_std = rlt::std(device, output);
    std::cout << "output_mean: " << output_mean << std::endl;
    std::cout << "output_std: " << output_std << std::endl;
    ASSERT_LT(rlt::math::abs(device.math, output_mean), 1e-10);
    ASSERT_LT(rlt::math::abs(device.math, 1 - output_std), 1e-6);

    for(TI dim_i=0; dim_i < DIM; dim_i++){
        T bias_value = rlt::get(bias, 0, dim_i);
        T variance_value = rlt::get(variance, 0, dim_i);
        T layer_mean = rlt::get(device, layer.mean.parameters, dim_i);
        T layer_std = 1/rlt::get(device, layer.precision.parameters, dim_i);
        std::cout << "mean: " << layer_mean << "/" << bias_value << " std:: " << layer_std << "/" << variance_value << std::endl;
        ASSERT_LT(rlt::math::abs(device.math, bias_value - layer_mean), 0.1);
        ASSERT_LT(rlt::math::abs(device.math, variance_value - layer_std), 0.1);
    }
}

