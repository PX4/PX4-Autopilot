#define RL_TOOLS_DISABLE_TENSORBOARD
#define RL_TOOLS_BACKEND_ENABLE_BLAS
//#define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#include <rl_tools/operations/cpu.h>
//#define RL_TOOLS_BACKEND_DISABLE_BLAS
#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <gtest/gtest.h>
#include <thread>
#include <chrono>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

namespace config{
    template <typename T_TYPE_POLICY, typename T_TI>
    struct CONFIG{
        using TYPE_POLICY = T_TYPE_POLICY;
        using T = typename TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        static constexpr TI SEQUENCE_LENGTH = 3;
        static constexpr TI INPUT_DIM = 4;
        static constexpr TI HIDDEN_DIM = 64;
        static constexpr TI OUTPUT_DIM = 1;
        static constexpr TI BATCH_SIZE = 64;
        static constexpr T THRESHOLD = 1e-5;

        using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
        using SPEC = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, 3, HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
        using MODEL = rlt::nn_models::mlp::NeuralNetwork<SPEC, CAPABILITY_ADAM, INPUT_SHAPE>;

        using LAYER_1_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_SPEC>;
        using LAYER_2_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_SPEC>;
        using LAYER_3_SPEC = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_SPEC>;

        using OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>>;
        using SEQUENTIAL_OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>>;

        template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
        using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
        using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;

        using SEQUENTIAL_MODEL = rlt::nn_models::sequential::Build<CAPABILITY_ADAM, MODULE_CHAIN, INPUT_SHAPE>;
    };
}




template <typename DEVICE, typename SEQUENTIAL_DEVICE, typename CONFIG>
void test_correctness(){
    typename CONFIG::MODEL model, model_temp;
    typename CONFIG::MODEL::template Buffer<> buffer;
    DEVICE device;
    SEQUENTIAL_DEVICE sdevice;
    typename CONFIG::SEQUENTIAL_MODEL sequential_model, sequential_model_temp;
    typename CONFIG::SEQUENTIAL_MODEL::template Buffer<> sequential_buffer;
    typename CONFIG::OPTIMIZER optimizer;
    typename CONFIG::SEQUENTIAL_OPTIMIZER sequential_optimizer;
    using T = typename CONFIG::T;
    using TI = typename CONFIG::TI;

    typename DEVICE::SPEC::RANDOM::template ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::INPUT_SHAPE>> input, d_input, d_input_sequential, d_input_only, d_input_sequential_only;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::OUTPUT_SHAPE>> output, output_eval, d_output, output_sequential, output_sequential_eval;

    rlt::malloc(device, optimizer);
    rlt::malloc(device, sequential_optimizer);
    rlt::malloc(device, input);
    rlt::malloc(device, d_input);
    rlt::malloc(device, d_input_only);
    rlt::malloc(sdevice, d_input_sequential);
    rlt::malloc(sdevice, d_input_sequential_only);
    rlt::malloc(device, output);
    rlt::malloc(device, output_eval);
    rlt::malloc(sdevice, output_sequential);
    rlt::malloc(sdevice, output_sequential_eval);
    rlt::malloc(device, d_output);
    rlt::malloc(device, model);
    rlt::malloc(device, model_temp);
    rlt::malloc(device, buffer);
    rlt::malloc(sdevice, sequential_model);
    rlt::malloc(sdevice, sequential_model_temp);
    rlt::malloc(sdevice, sequential_buffer);

    rlt::init(device, optimizer);
    rlt::init(device, sequential_optimizer);

    rlt::init_weights(device, model, rng);
    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);

    rlt::copy(device, sdevice, model.input_layer, sequential_model.content);
    rlt::copy(device, sdevice, model.hidden_layers[0], sequential_model.next_module.content);
    rlt::copy(device, sdevice, model.output_layer, sequential_model.next_module.next_module.content);

    rlt::evaluate(device, model, input, output_eval, buffer, rng);
    rlt::evaluate(sdevice, sequential_model, input, output_sequential_eval, sequential_buffer, rng);

    {
        auto abs_diff = rlt::abs_diff(device, output_eval, output_sequential_eval);
        std::cout << "abs_diff evaluate: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    rlt::forward(device, model, input, output, buffer, rng);
    rlt::forward(sdevice, sequential_model, input, output_sequential, sequential_buffer, rng);

    {
        auto abs_diff = rlt::abs_diff(device, output, output_sequential);
        std::cout << "abs_diff forward: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    rlt::reset_optimizer_state(device, optimizer, model);
    rlt::reset_optimizer_state(sdevice, sequential_optimizer, sequential_model);
    rlt::zero_gradient(device, model);
    rlt::zero_gradient(sdevice, sequential_model);
    rlt::backward(device, model, input, d_output, buffer);
    rlt::backward(sdevice, sequential_model, input, d_output, sequential_buffer);

    {
        auto abs_diff = rlt::abs_diff(device, model.input_layer, sequential_model.content);
        abs_diff += rlt::abs_diff(device, model.hidden_layers[0], sequential_model.next_module.content);
        abs_diff += rlt::abs_diff(device, model.output_layer, sequential_model.next_module.next_module.content);
        std::cout << "abs_diff gradient: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    rlt::copy(device, device, model, model_temp);
    rlt::copy(sdevice, sdevice, sequential_model, sequential_model_temp);

    rlt::step(device, optimizer, model);
    rlt::step(sdevice, sequential_optimizer, sequential_model);

    {
        auto abs_diff = rlt::abs_diff(device, model.input_layer, sequential_model.content);
        abs_diff += rlt::abs_diff(device, model.hidden_layers[0], sequential_model.next_module.content);
        abs_diff += rlt::abs_diff(device, model.output_layer, sequential_model.next_module.next_module.content);
        std::cout << "abs_diff adam step: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    {
        auto abs_diff = rlt::abs_diff(device, model, model_temp);
        abs_diff += rlt::abs_diff(device, sequential_model, sequential_model_temp);
        std::cout << "abs_diff pre-post: " << abs_diff << std::endl;
        ASSERT_GT(abs_diff, 0);
    }

    rlt::reset_forward_state(device, model);
    rlt::reset_forward_state(sdevice, sequential_model);

    {
        auto abs_diff = rlt::abs_diff(device, model.input_layer, sequential_model.content);
        abs_diff += rlt::abs_diff(device, model.hidden_layers[0], sequential_model.next_module.content);
        abs_diff += rlt::abs_diff(device, model.output_layer, sequential_model.next_module.next_module.content);
        std::cout << "abs diff reset forward state: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    rlt::reset_optimizer_state(device, optimizer, model);
    rlt::reset_optimizer_state(sdevice, sequential_optimizer, sequential_model);
    rlt::zero_gradient(device, model);
    rlt::zero_gradient(sdevice, sequential_model);
    rlt::forward(device, model, input, output, buffer, rng);
    rlt::forward(sdevice, sequential_model, input, output_sequential, sequential_buffer, rng);
    rlt::backward_full(device, model, input, d_output, d_input, buffer);
    rlt::backward_full(sdevice, sequential_model, input, d_output, d_input_sequential, sequential_buffer);

    {
        auto abs_diff = rlt::abs_diff(device, model.input_layer, sequential_model.content);
        abs_diff += rlt::abs_diff(device, model.hidden_layers[0], sequential_model.next_module.content);
        abs_diff += rlt::abs_diff(device, model.output_layer, sequential_model.next_module.next_module.content);
        std::cout << "abs_diff gradient full: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }
    {
        auto abs_diff = rlt::abs_diff(device, d_input, d_input_sequential);
        std::cout << "abs_diff d_input: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    rlt::copy(device, device, model, model_temp);
    rlt::copy(sdevice, sdevice, sequential_model, sequential_model_temp);

    rlt::backward_input(device, model, d_output, d_input_only, buffer);
    rlt::backward_input(sdevice, sequential_model, d_output, d_input_sequential_only, sequential_buffer);

    {
        auto abs_diff = rlt::abs_diff(device, d_input_only, d_input_sequential_only);
        std::cout << "abs_diff d_input only: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }
    {
        auto abs_diff = rlt::abs_diff(device, d_input_only, d_input);
        std::cout << "abs_diff d_input pre-post: " << abs_diff << std::endl;
        ASSERT_LT(abs_diff, CONFIG::THRESHOLD);
    }

    {
        auto abs_diff = rlt::abs_diff(device, model, model_temp);
        abs_diff += rlt::abs_diff(device, sequential_model, sequential_model_temp);
        std::cout << "abs_diff pre-post: " << abs_diff << std::endl;
        ASSERT_EQ(abs_diff, 0);
    }

}
TEST(RL_TOOLS_NN_LAYERS_DENSE, CORRECTNESS_BACKWARD_PARAMS_BLAS){
    using TYPE_POLICY = rlt::numeric_types::Policy<float>;
//using DEVICE = rlt::devices::DefaultCPU;
    using DEVICE = rlt::devices::DEVICE_FACTORY<rlt::devices::DefaultCPUSpecification>;
    using TI = typename DEVICE::index_t;

    test_correctness<DEVICE, DEVICE, config::CONFIG<TYPE_POLICY, TI>>();
}

TEST(RL_TOOLS_NN_LAYERS_DENSE, CORRECTNESS_BACKWARD_PARAMS_BLAS_CPU){
    using TYPE_POLICY = rlt::numeric_types::Policy<double>;
//using DEVICE = rlt::devices::DefaultCPU;
    using DEVICE = rlt::devices::DEVICE_FACTORY<rlt::devices::DefaultCPUSpecification>;
    using SEQUENTIAL_DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;

    test_correctness<DEVICE, SEQUENTIAL_DEVICE, config::CONFIG<TYPE_POLICY, TI>>();
}

TEST(RL_TOOLS_NN_LAYERS_DENSE, CORRECTNESS_BACKWARD_PARAMS_CPU_BLAS){
    using TYPE_POLICY = rlt::numeric_types::Policy<double>;
//using DEVICE = rlt::devices::DefaultCPU;
    using DEVICE = rlt::devices::DEVICE_FACTORY<rlt::devices::DefaultCPUSpecification>;
    using SEQUENTIAL_DEVICE = rlt::devices::DefaultCPU;
    using TI = typename DEVICE::index_t;

    test_correctness<SEQUENTIAL_DEVICE, DEVICE, config::CONFIG<TYPE_POLICY, TI>>();
}
