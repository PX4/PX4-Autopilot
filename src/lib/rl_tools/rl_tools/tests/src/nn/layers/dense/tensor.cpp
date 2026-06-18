#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using TI = typename DEVICE::index_t;




#include <gtest/gtest.h>



TEST(RL_TOOLS_NN_LAYERS_DENSE_TENSOR, MAIN) {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    using T = float;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 10, 15>>> tensor;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 10, 15>> matrix;

    rlt::malloc(device, tensor);
    rlt::malloc(device, matrix);
    rlt::randn(device, matrix, rng);


    *rlt::data_pointer(tensor) = matrix._data;

    auto matrix_view = rlt::matrix_view(device, tensor);

    rlt::print(device, matrix);
    rlt::print(device, tensor);

    auto abs_diff = rlt::abs_diff(device, matrix, matrix_view);
    std::cout << "abs_diff: " << abs_diff << std::endl;

    ASSERT_NEAR(abs_diff, 0, 1e-5);

}

TEST(RL_TOOLS_NN_LAYERS_DENSE_TENSOR, ND_Tensor){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    using T = float;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 2, 10, 15>>> tensor;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 20, 15>> matrix;

    rlt::malloc(device, tensor);
    rlt::malloc(device, matrix);
    rlt::randn(device, matrix, rng);


    *rlt::data_pointer(tensor) = matrix._data;

    auto matrix_view = rlt::matrix_view(device, tensor);

    rlt::print(device, matrix);
    rlt::print(device, tensor);

    auto abs_diff = rlt::abs_diff(device, matrix, matrix_view);
    std::cout << "abs_diff: " << abs_diff << std::endl;

    ASSERT_NEAR(abs_diff, 0, 1e-5);
}

template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
TEST(RL_TOOLS_NN_LAYERS_DENSE_TENSOR, FORWARD){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    constexpr TI INPUT_DIM = 10;
    constexpr TI OUTPUT_DIM = 20;
    constexpr TI BATCH_SIZE = 5;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
    using LAYER_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER = rlt::nn::layers::dense::BindConfiguration<LAYER_SPEC>;
    using CAPA = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;

    using MODULE_CHAIN = Module<LAYER>;
    using MODEL = rlt::nn_models::sequential::Build<CAPA, MODULE_CHAIN, INPUT_SHAPE>;
    MODEL model;
    MODEL::Buffer<> buffer;

    constexpr TI FIRST_DIM = 2;
    constexpr TI SECOND_DIM = 2;

    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, FIRST_DIM, SECOND_DIM, INPUT_DIM>>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, FIRST_DIM, SECOND_DIM, OUTPUT_DIM>>> output;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, FIRST_DIM, SECOND_DIM, OUTPUT_DIM>>> output2;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, output2);
    rlt::malloc(device, model);

    rlt::randn(device, input, rng);
    rlt::init_weights(device, model, rng);

    auto matrix_input = matrix_view(device, input);
    auto matrix_output = matrix_view(device, output);
    auto matrix_output2 = matrix_view(device, output2);

    for(TI i = 0; i < FIRST_DIM; i++){
        for(TI j = 0; j < SECOND_DIM; j++){
            for(TI k = 0; k < INPUT_DIM; k++){
                T matrix_value = rlt::get(matrix_input, i*SECOND_DIM + j, k);
                T tensor_value = rlt::get(device, input, i, j, k);
                ASSERT_EQ(matrix_value, tensor_value);
            }
        }
    }
    rlt::evaluate(device, model, input, output, buffer, rng);
    rlt::evaluate(device, model.content, matrix_input, matrix_output2, buffer.content_buffer.buffer, rng);

    T abs_diff = rlt::abs_diff(device, matrix_output, matrix_output2);
    std::cout << "abs_diff: " << abs_diff << std::endl;
    ASSERT_NEAR(abs_diff, 0, 1e-5);
}
