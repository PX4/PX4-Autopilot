#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <gtest/gtest.h>


//template <typename T_CONTENT>
//struct OutputModule{
//    using CONTENT = T_CONTENT;
//    static constexpr auto MAX_HIDDEN_DIM = CONTENT::INPUT_DIM;
//    CONTENT content;
//};
//
//template <typename T_CONTENT, typename T_NEXT_MODULE>
//struct Specification{
//    using CONTENT = T_CONTENT;
//    using NEXT_MODULE = T_NEXT_MODULE;
//    static constexpr auto NEXT_MODULE_INPUT_DIM = NEXT_MODULE::CONTENT::INPUT_DIM;
//    static_assert(NEXT_MODULE_INPUT_DIM == CONTENT::OUTPUT_DIM);
//    static constexpr auto NEXT_MODULE_INPUT_DIM = NEXT_MODULE::CONTENT::INPUT_DIM;
//};
//


template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_SEQUENTIAL_STATIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;


    constexpr TI BATCH_SIZE = 1;
    {
        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 20>;

        using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
        

        using MODULE_CHAIN = Module<LAYER_1>;
        using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, INPUT_SHAPE>;

        static_assert(rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() == 0);
        static_assert(SEQUENTIAL::SPEC::MAX_HIDDEN_DIM == 0);
        static_assert(rlt::get<2>(typename SEQUENTIAL::OUTPUT_SHAPE{}) == 10);
    }
    {

        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 20>;
        using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
        using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 1, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
        
        using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2>>;
        using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, INPUT_SHAPE>;

        static_assert(rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() == 10);
        static_assert(SEQUENTIAL::SPEC::MAX_HIDDEN_DIM == 10);
        static_assert(rlt::get<2>(typename SEQUENTIAL::OUTPUT_SHAPE{}) == 1);
    }
    {
        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 20>;
        using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
        using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 100, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;

        using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2>>;
        using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, INPUT_SHAPE>;

        static_assert(rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() == 10);
        static_assert(SEQUENTIAL::SPEC::MAX_HIDDEN_DIM == 10);
        static_assert(rlt::get<2>(typename SEQUENTIAL::OUTPUT_SHAPE{}) == 100);
    }
    {
        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 20>;
        using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
        using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 11, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
        using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 11, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;
        using LAYER_4_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 20, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_4 = rlt::nn::layers::dense::BindConfiguration<LAYER_4_CONFIG>;

        using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3, Module<LAYER_4>>>>;
        using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, INPUT_SHAPE>;

        static_assert(rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() == 11);
        static_assert(SEQUENTIAL::SPEC::MAX_HIDDEN_DIM == 11);
        static_assert(rlt::get<2>(typename SEQUENTIAL::OUTPUT_SHAPE{}) == 20);
    }
    {
        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 20>;
        using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
        using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 11, rlt::nn::activation_functions::ActivationFunction::RELU>;
        using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
        using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 11, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;
        using LAYER_4_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 100, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_4 = rlt::nn::layers::dense::BindConfiguration<LAYER_4_CONFIG>;
        using LAYER_5_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 20, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
        using LAYER_5 = rlt::nn::layers::dense::BindConfiguration<LAYER_5_CONFIG>;
        
        using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3, Module<LAYER_4, Module<LAYER_5>>>>>;
        using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Forward<>, MODULE_CHAIN, INPUT_SHAPE>;

        static_assert(rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() == 100);
        static_assert(SEQUENTIAL::SPEC::MAX_HIDDEN_DIM == 100);
        static_assert(rlt::get<2>(typename SEQUENTIAL::OUTPUT_SHAPE{}) == 20);
    }

}

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_FORWARD){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;

    constexpr TI BATCH_SIZE = 1;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using MLP_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 2, 3, 10, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;
    using MLP = rlt::nn_models::mlp::NeuralNetwork<MLP_CONFIG, CAPABILITY_ADAM, INPUT_SHAPE>;

    using LAYER_1_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
    using LAYER_2_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
    using LAYER_3_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 2, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;

    std::cout << "Max hidden dim: " << rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() << std::endl;

    DEVICE device;
    MLP mlp;
    MLP::Buffer<> mlp_buffer;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    LAYER_1::Layer<CAPABILITY_ADAM, LAYER_1_INPUT_SHAPE> layer_1;
    LAYER_2::Layer<CAPABILITY_ADAM, LAYER_2_INPUT_SHAPE> layer_2;
    LAYER_3::Layer<CAPABILITY_ADAM, LAYER_3_INPUT_SHAPE> layer_3;
    decltype(layer_1)::Buffer<> layer_1_buffer;
    decltype(layer_2)::Buffer<> layer_2_buffer;
    decltype(layer_3)::Buffer<> layer_3_buffer;

    SEQUENTIAL sequential;
    SEQUENTIAL::Buffer<BATCH_SIZE> sequential_buffer;

    rlt::malloc(device, mlp);
    rlt::malloc(device, mlp_buffer);
    rlt::malloc(device, layer_1);
    rlt::malloc(device, layer_1_buffer);
    rlt::malloc(device, layer_2);
    rlt::malloc(device, layer_2_buffer);
    rlt::malloc(device, layer_3);
    rlt::malloc(device, layer_3_buffer);

    rlt::malloc(device, sequential);
    rlt::malloc(device, sequential_buffer);

    rlt::init_weights(device, mlp, rng);
    rlt::copy(device, device, mlp.input_layer, layer_1);
    rlt::copy(device, device, mlp.hidden_layers[0], layer_2);
    rlt::copy(device, device, mlp.output_layer, layer_3);

    rlt::copy(device, device, mlp.input_layer, sequential.content);
    rlt::copy(device, device, mlp.hidden_layers[0], sequential.next_module.content);
    rlt::copy(device, device, mlp.output_layer, sequential.next_module.next_module.content);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> hidden_tick;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> hidden_tock;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_mlp;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_chain;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_sequential;
    rlt::malloc(device, input);
    rlt::malloc(device, hidden_tick);
    rlt::malloc(device, hidden_tock);
    rlt::malloc(device, output_mlp);
    rlt::malloc(device, output_chain);
    rlt::malloc(device, output_sequential);

    rlt::randn(device, input, rng);
    rlt::print(device, input);

    for(TI i = 0; i < 2; i++){
        rlt::forward(device, mlp, input, output_mlp, mlp_buffer, rng);
        rlt::print(device, output_mlp);

        rlt::forward(device, layer_1, input, hidden_tick, layer_1_buffer, rng);
        rlt::forward(device, layer_2, hidden_tick, hidden_tock, layer_2_buffer, rng);
        rlt::forward(device, layer_3, hidden_tock, output_chain, layer_3_buffer, rng);
        rlt::print(device, output_chain);

        rlt::forward(device, sequential.content                        , input, hidden_tick, layer_1_buffer, rng);
        rlt::forward(device, sequential.next_module.content            , hidden_tick, hidden_tock, layer_2_buffer, rng);
        rlt::forward(device, sequential.next_module.next_module.content, hidden_tock, output_sequential, layer_3_buffer, rng);
        rlt::print(device, output_sequential);

        auto output_sequential_tensor_ravel = rlt::to_tensor(device, output_sequential);
        auto output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, output_sequential_tensor_ravel);
        rlt::forward(device, sequential, input, output_sequential_tensor, sequential_buffer, rng);
        rlt::print(device, output_sequential);

        auto abs_diff_sequential = rlt::abs_diff(device, output_mlp, output_sequential);
        auto abs_diff_chain = rlt::abs_diff(device, output_mlp, output_sequential);

        std::cout << "Abs diff sequential: " << abs_diff_sequential << std::endl;
        std::cout << "Abs diff chain: " << abs_diff_chain << std::endl;

        ASSERT_LT(abs_diff_sequential, 1e-8);
        ASSERT_LT(abs_diff_chain, 1e-8);

        rlt::init_weights(device, sequential, rng);
        rlt::copy(device, device, sequential.content, mlp.input_layer);
        rlt::copy(device, device, sequential.next_module.content, mlp.hidden_layers[0]);
        rlt::copy(device, device, sequential.next_module.next_module.content, mlp.output_layer);
    }
}

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_INCOMPATIBLE_DEFINITION){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;
    constexpr TI BATCH_SIZE = 10;


    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 3>;
    using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 1, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;

    DEVICE device;
    SEQUENTIAL model;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    rlt::malloc(device, model);
    rlt::init_weights(device, model, rng);
}

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_EVALUATE){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;
    constexpr TI BATCH_SIZE = 1;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using LAYER_1_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
    using LAYER_2_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
    using LAYER_3_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 2, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;

    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;


    std::cout << "Max hidden dim: " << rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() << std::endl;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    LAYER_1::Layer<CAPABILITY_ADAM, LAYER_1_INPUT_SHAPE> layer_1;
    LAYER_2::Layer<CAPABILITY_ADAM, LAYER_2_INPUT_SHAPE> layer_2;
    LAYER_3::Layer<CAPABILITY_ADAM, LAYER_3_INPUT_SHAPE> layer_3;

    SEQUENTIAL sequential;
    typename SEQUENTIAL::Buffer<1> sequential_buffer;

    rlt::malloc(device, sequential);
    rlt::malloc(device, sequential_buffer);
    rlt::init_weights(device, sequential, rng);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_sequential;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_sequential_evaluate;
    rlt::malloc(device, input);
    rlt::malloc(device, output_sequential);
    rlt::malloc(device, output_sequential_evaluate);

    rlt::randn(device, input, rng);
    rlt::print(device, input);

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;

    auto output_sequential_tensor_ravel = rlt::to_tensor(device, output_sequential);
    auto output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, output_sequential_tensor_ravel);
    rlt::forward(device, sequential, input, output_sequential_tensor, sequential_buffer, rng);
    rlt::print(device, output_sequential);
    auto output_sequential_evaluate_tensor_ravel = rlt::to_tensor(device, output_sequential_evaluate);
    auto output_sequential_evaluate_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, output_sequential_evaluate_tensor_ravel);
    auto input_tensor_ravel = rlt::to_tensor(device, input);
    auto input_tensor = rlt::view_memory<INPUT_SHAPE>(device, input_tensor_ravel);
    rlt::evaluate(device, sequential, input_tensor, output_sequential_evaluate_tensor, sequential_buffer, rng);
    rlt::print(device, output_sequential_evaluate);

    auto abs_diff = rlt::abs_diff(device, output_sequential_evaluate, output_sequential);

    std::cout << "Abs diff evaluate: " << abs_diff << std::endl;

    ASSERT_LT(abs_diff, 1e-8);

}

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_BACKWARD){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;
    constexpr TI BATCH_SIZE = 1;
    constexpr T THRESHOLD = 1e-8;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;
    using MLP_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, 2, 3, 10, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using MLP = rlt::nn_models::mlp::NeuralNetwork<MLP_CONFIG, CAPABILITY_ADAM, INPUT_SHAPE>;

    using LAYER_1_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
    using LAYER_2_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
    using LAYER_3_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 2, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;

    std::cout << "Max hidden dim: " << rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() << std::endl;

    DEVICE device;
    MLP mlp;
    typename MLP::Buffer<1> mlp_buffers;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    LAYER_1::Layer<CAPABILITY_ADAM, LAYER_1_INPUT_SHAPE> layer_1;
    LAYER_2::Layer<CAPABILITY_ADAM, LAYER_2_INPUT_SHAPE> layer_2;
    LAYER_3::Layer<CAPABILITY_ADAM, LAYER_3_INPUT_SHAPE> layer_3;

    decltype(layer_3)::Buffer<1> layer_buffer;

    SEQUENTIAL sequential;
    SEQUENTIAL::Buffer<1> buffer_sequential;

    rlt::malloc(device, mlp);
    rlt::malloc(device, layer_1);
    rlt::malloc(device, layer_2);
    rlt::malloc(device, layer_3);
    rlt::malloc(device, mlp_buffers);

    rlt::malloc(device, sequential);
    rlt::malloc(device, buffer_sequential);

    rlt::init_weights(device, mlp, rng);
    rlt::copy(device, device, mlp.input_layer, layer_1);
    rlt::copy(device, device, mlp.hidden_layers[0], layer_2);
    rlt::copy(device, device, mlp.output_layer, layer_3);

    rlt::copy(device, device, mlp.input_layer, sequential.content);
    rlt::copy(device, device, mlp.hidden_layers[0], sequential.next_module.content);
    rlt::copy(device, device, mlp.output_layer, sequential.next_module.next_module.content);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> d_input_mlp, d_input_chain, d_input_sequential;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> hidden_tick;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> hidden_tock;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> d_hidden_tick;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 10>> d_hidden_tock;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_mlp;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_chain;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_sequential;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> d_output;
    rlt::malloc(device, input);
    rlt::malloc(device, d_input_mlp);
    rlt::malloc(device, d_input_chain);
    rlt::malloc(device, d_input_sequential);
    rlt::malloc(device, hidden_tick);
    rlt::malloc(device, hidden_tock);
    rlt::malloc(device, d_hidden_tick);
    rlt::malloc(device, d_hidden_tock);
    rlt::malloc(device, output_mlp);
    rlt::malloc(device, output_chain);
    rlt::malloc(device, output_sequential);
    rlt::malloc(device, d_output);

    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);
    rlt::print(device, input);

    rlt::forward(device, mlp, input, mlp_buffers, output_mlp);
    rlt::zero_gradient(device, mlp);
    rlt::backward_full(device, mlp, input, d_output, d_input_mlp, mlp_buffers);

    rlt::print(device, d_input_mlp);

    rlt::zero_gradient(device, layer_1);
    rlt::zero_gradient(device, layer_2);
    rlt::zero_gradient(device, layer_3);
    rlt::forward(device, layer_1, input, hidden_tick, layer_buffer, rng);
    rlt::forward(device, layer_2, hidden_tick, hidden_tock, layer_buffer, rng);
    rlt::forward(device, layer_3, hidden_tock, output_chain, layer_buffer, rng);
    rlt::backward_full(device, layer_3, hidden_tock, d_output, d_hidden_tick, layer_buffer);
    rlt::backward_full(device, layer_2, hidden_tick, d_hidden_tick, d_hidden_tock, layer_buffer);
    rlt::backward_full(device, layer_1, input, d_hidden_tock, d_input_chain, layer_buffer);

    rlt::print(device, d_input_chain);

    {
        auto abs_diff_d_input = rlt::abs_diff(device, d_input_mlp, d_input_chain);
        auto abs_diff_grad_W_1 = rlt::abs_diff(device, mlp.input_layer.weights.gradient, layer_1.weights.gradient);
        auto abs_diff_grad_b_1 = rlt::abs_diff(device, mlp.input_layer.biases.gradient, layer_1.biases.gradient);
        auto abs_diff_grad_W_2 = rlt::abs_diff(device, mlp.hidden_layers[0].weights.gradient, layer_2.weights.gradient);
        auto abs_diff_grad_b_2 = rlt::abs_diff(device, mlp.hidden_layers[0].biases.gradient, layer_2.biases.gradient);
        auto abs_diff_grad_W_3 = rlt::abs_diff(device, mlp.output_layer.weights.gradient, layer_3.weights.gradient);
        auto abs_diff_grad_b_3 = rlt::abs_diff(device, mlp.output_layer.biases.gradient, layer_3.biases.gradient);

        ASSERT_LT(abs_diff_d_input, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_1, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_1, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_2, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_2, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_3, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_3, THRESHOLD);
    }

    rlt::forward(device, sequential.content                        , input, hidden_tick, layer_buffer, rng);
    rlt::forward(device, sequential.next_module.content            , hidden_tick, hidden_tock, layer_buffer, rng);
    rlt::forward(device, sequential.next_module.next_module.content, hidden_tock, output_sequential, layer_buffer, rng);

    rlt::set(device, sequential.content.weights.gradient, 10, 0, 0);
    rlt::set(device, sequential.next_module.content.weights.gradient, 10, 0, 0);
    rlt::set(device, sequential.next_module.next_module.content.weights.gradient, 10, 0, 0);
    auto output_sequential_tensor_ravel = rlt::to_tensor(device, output_sequential);
    auto output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, output_sequential_tensor_ravel);
    rlt::forward(device, sequential, input, output_sequential_tensor, buffer_sequential, rng);
    rlt::zero_gradient(device, sequential);
    auto input_tensor_ravel = rlt::to_tensor(device, input);
    auto input_tensor = rlt::view_memory<INPUT_SHAPE>(device, input_tensor_ravel);
    auto d_output_sequential_tensor_ravel = rlt::to_tensor(device, d_output);
    auto d_output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, d_output_sequential_tensor_ravel);
    auto d_input_sequential_tensor_ravel = rlt::to_tensor(device, d_input_sequential);
    auto d_input_sequential_tensor = rlt::view_memory<INPUT_SHAPE>(device, d_input_sequential_tensor_ravel);
    rlt::backward_full(device, sequential, input_tensor, d_output_sequential_tensor, d_input_sequential_tensor, buffer_sequential);

    rlt::print(device, d_input_sequential);

    {
        auto abs_diff_d_input = rlt::abs_diff(device, d_input_mlp, d_input_chain);
        auto abs_diff_grad_W_1 = rlt::abs_diff(device, sequential.content.weights.gradient, layer_1.weights.gradient);
        auto abs_diff_grad_b_1 = rlt::abs_diff(device, sequential.content.biases.gradient, layer_1.biases.gradient);
        auto abs_diff_grad_W_2 = rlt::abs_diff(device, sequential.next_module.content.weights.gradient, layer_2.weights.gradient);
        auto abs_diff_grad_b_2 = rlt::abs_diff(device, sequential.next_module.content.biases.gradient, layer_2.biases.gradient);
        auto abs_diff_grad_W_3 = rlt::abs_diff(device, sequential.next_module.next_module.content.weights.gradient, layer_3.weights.gradient);
        auto abs_diff_grad_b_3 = rlt::abs_diff(device, sequential.next_module.next_module.content.biases.gradient, layer_3.biases.gradient);

        ASSERT_LT(abs_diff_d_input, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_1, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_1, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_2, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_2, THRESHOLD);
        ASSERT_LT(abs_diff_grad_W_3, THRESHOLD);
        ASSERT_LT(abs_diff_grad_b_3, THRESHOLD);
    }
}

TEST(RL_TOOLS_NN_MODELS_MLP_SEQUENTIAL, TEST_BACKWARD_2){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = typename DEVICE::index_t;
    constexpr TI BATCH_SIZE = 1;
    constexpr T THRESHOLD = 1e-8;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, BATCH_SIZE>;
    using LAYER_1_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 5>;
    using LAYER_1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 10, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_1 = rlt::nn::layers::dense::BindConfiguration<LAYER_1_CONFIG>;
    using LAYER_2_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 10>;
    using LAYER_2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 6, rlt::nn::activation_functions::ActivationFunction::RELU>;
    using LAYER_2 = rlt::nn::layers::dense::BindConfiguration<LAYER_2_CONFIG>;
    using LAYER_3_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, 6>;
    using LAYER_3_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, 2, rlt::nn::activation_functions::ActivationFunction::IDENTITY>;
    using LAYER_3 = rlt::nn::layers::dense::BindConfiguration<LAYER_3_CONFIG>;

    using MODULE_CHAIN = Module<LAYER_1, Module<LAYER_2, Module<LAYER_3>>>;
    using SEQUENTIAL = rlt::nn_models::sequential::Build<rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, MODULE_CHAIN, INPUT_SHAPE>;

    std::cout << "Max hidden dim: " << rlt::nn_models::sequential::find_max_hiddend_dim<TI, typename SEQUENTIAL::SPEC>() << std::endl;

    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 1);

    LAYER_1::Layer<CAPABILITY_ADAM, LAYER_1_INPUT_SHAPE> layer_1;
    LAYER_2::Layer<CAPABILITY_ADAM, LAYER_2_INPUT_SHAPE> layer_2;
    LAYER_3::Layer<CAPABILITY_ADAM, LAYER_3_INPUT_SHAPE> layer_3;

    decltype(layer_3)::Buffer<1> layer_buffer;

    SEQUENTIAL sequential;
    SEQUENTIAL::Buffer<1> buffer_sequential;

    rlt::malloc(device, sequential);
    rlt::malloc(device, buffer_sequential);

    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> input;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 5>> d_input_sequential;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> output_sequential;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, 2>> d_output;
    rlt::malloc(device, input);
    rlt::malloc(device, d_input_sequential);
    rlt::malloc(device, output_sequential);
    rlt::malloc(device, d_output);
    rlt::init_weights(device, sequential, rng);

    rlt::randn(device, input, rng);
    rlt::randn(device, d_output, rng);
    rlt::print(device, input);

    auto output_sequential_tensor_ravel = rlt::to_tensor(device, output_sequential);
    auto output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, output_sequential_tensor_ravel);
    auto input_tensor_ravel = rlt::to_tensor(device, input);
    auto input_tensor = rlt::view_memory<INPUT_SHAPE>(device, input_tensor_ravel);
    rlt::forward(device, sequential, input_tensor, output_sequential_tensor, buffer_sequential, rng);
    rlt::zero_gradient(device, sequential);
    auto d_output_sequential_tensor_ravel = rlt::to_tensor(device, d_output);
    auto d_output_sequential_tensor = rlt::view_memory<SEQUENTIAL::OUTPUT_SHAPE>(device, d_output_sequential_tensor_ravel);
    auto d_input_sequential_tensor_ravel = rlt::to_tensor(device, d_input_sequential);
    auto d_input_sequential_tensor = rlt::view_memory<INPUT_SHAPE>(device, d_input_sequential_tensor_ravel);
    rlt::backward_full(device, sequential, input_tensor, d_output_sequential_tensor, d_input_sequential_tensor, buffer_sequential);

    rlt::print(device, d_input_sequential);
}
