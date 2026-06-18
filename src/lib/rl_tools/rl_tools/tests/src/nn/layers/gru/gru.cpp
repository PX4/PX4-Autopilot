#include <gtest/gtest.h>
#include <rl_tools/operations/cpu.h>

#include <rl_tools/containers/tensor/operations_generic.h>
#include <rl_tools/containers/tensor/operations_cpu.h>


#include <rl_tools/nn/layers/gru/operations_generic.h>

#include <rl_tools/nn_models/sequential/operations_generic.h>

//#include <rl_tools/nn/layers/gru/layer.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>

namespace rlt = rl_tools;


#include "../../../utils/utils.h"


struct SPEC{};
using RESET_MODE = rlt::Mode<rlt::nn::layers::gru::ResetMode<rlt::mode::Default<>, rlt::nn::layers::gru::ResetModeSpecification<int, int>>>;
static_assert(rlt::nn::layers::gru::mode::can_reset_sample<void>(RESET_MODE{}) == true);
using DEFAULT_MODE = rlt::Mode<rlt::mode::Default<>>;
static_assert(rlt::nn::layers::gru::mode::can_reset_sample<void>(DEFAULT_MODE{}) == false);

TEST(RL_TOOLS_NN_LAYERS_GRU, MATRIX_MULTIPLICATION_TRANSPOSE_GENERIC){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = DEVICE::index_t;
    DEVICE device;
    using SHAPE = rlt::tensor::Shape<TI, 2, 2>;
    using STRIDE = rlt::tensor::RowMajorStride<SHAPE>;
    using SHAPE_TRANSPOSE = rlt::tensor::Shape<TI, rlt::get<1>(SHAPE{}), rlt::get<0>(SHAPE{})>;
    using STRIDE_TRANSPOSE = rlt::tensor::Stride<TI, rlt::get<1>(STRIDE{}), rlt::get<0>(STRIDE{})>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE>> A, B, C, C_target;
    rlt::Tensor<rlt::tensor::Specification<T, TI, SHAPE, true, STRIDE_TRANSPOSE>> A_T, B_T, C_T, C_target_T;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 2>>> bias;
    rlt::malloc(device, A);
    rlt::malloc(device, B);
    rlt::malloc(device, C);
    rlt::malloc(device, C_target);
    rlt::malloc(device, bias);
    rlt::set_all(device, bias, 0);
    rlt::set(device, bias, 1, 0);
    rlt::set(device, bias, 3, 1);
    *rlt::data_pointer(A_T) = rlt::data(A);
    *rlt::data_pointer(B_T) = rlt::data(B);
    *rlt::data_pointer(C_T) = rlt::data(C);
    *rlt::data_pointer(C_target_T) = rlt::data(C_target);

    rlt::set(device, A, -0.259093, 0, 0);
    rlt::set(device, A, -1.498961, 0, 1);
    rlt::set(device, A, +0.119264, 1, 0);
    rlt::set(device, A, +0.458181, 1, 1);

    rlt::set(device, B, +0.394975, 0, 0);
    rlt::set(device, B, +0.044197, 0, 1);
    rlt::set(device, B, -0.636256, 1, 0);
    rlt::set(device, B, +1.731264, 1, 1);

    rlt::set(device, C_target, -0.259093 * +0.394975 + -1.498961 * -0.636256 + 1, 0, 0);
    rlt::set(device, C_target, -0.259093 * +0.044197 + -1.498961 * +1.731264 + 1, 0, 1);
    rlt::set(device, C_target, +0.119264 * +0.394975 + +0.458181 * -0.636256 + 3, 1, 0);
    rlt::set(device, C_target, +0.119264 * +0.044197 + +0.458181 * +1.731264 + 3, 1, 1);
    rlt::print(device, C_target);

//    rlt::multiply(device, A, B, C);
    rlt::nn::layers::gru::helper::matrix_multiply_transpose_bias(device, A, B_T, bias, C_T);
    rlt::print(device, C);
    auto diff = rlt::abs_diff(device, C_target, C);
    std::cout << "Matrix mul diff: " << diff << std::endl;
    ASSERT_TRUE(diff < 1e-15);
    rlt::free(device, A);
    rlt::free(device, B);
    rlt::free(device, C);
    rlt::free(device, C_target);
    rlt::free(device, bias);
}


template <bool USE_RESET_MODE>
void test_loading(std::string DATA_FILE_NAME){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI = DEVICE::index_t;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    constexpr T EPSILON = 1e-10;
    constexpr TI SEQUENCE_LENGTH = 50;
    constexpr TI BATCH_SIZE = 128;
    constexpr TI INPUT_DIM = 1;
    constexpr TI OUTPUT_DIM = 1;
    constexpr TI HIDDEN_DIM = 16;
    using RESET_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>;
    rlt::Tensor<rlt::tensor::Specification<bool, TI, RESET_SHAPE>> reset;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, INPUT_SHAPE>> input, dinput, dinput_real;
    using GRU_OUTPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_OUTPUT_SHAPE>> gru_output, dloss_dgru_output;
    using GRU_OUTPUT_STEP_SHAPE = rlt::tensor::Shape<TI, BATCH_SIZE, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_OUTPUT_STEP_SHAPE>> dloss_dgru_output_step;
    using GRU_INPUT_WEIGHT_SHAPE = rlt::tensor::Shape<TI, HIDDEN_DIM, INPUT_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_INPUT_WEIGHT_SHAPE>> grad_W_ir, grad_W_iz, grad_W_in;
    using GRU_INPUT_BIAS_SHAPE = rlt::tensor::Shape<TI, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_INPUT_BIAS_SHAPE>> grad_b_ir, grad_b_iz, grad_b_in;
    using GRU_HIDDEN_WEIGHT_SHAPE = rlt::tensor::Shape<TI, HIDDEN_DIM, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_HIDDEN_WEIGHT_SHAPE>> grad_W_hr, grad_W_hz, grad_W_hn;
    using GRU_HIDDEN_BIAS_SHAPE = rlt::tensor::Shape<TI, HIDDEN_DIM>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, GRU_HIDDEN_BIAS_SHAPE>> grad_b_hr, grad_b_hz, grad_b_hn;

    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, rlt::nn::parameters::Gradient>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, INPUT_DIM>;
    rlt::nn::layers::gru::Layer<GRU_CONFIG, CAPABILITY, INPUT_SHAPE> gru;
    decltype(gru)::Buffer<true> buffer;
    rlt::malloc(device, gru);
    rlt::malloc(device, buffer);

    rlt::malloc(device, reset);
    rlt::malloc(device, input);
    rlt::malloc(device, dinput);
    rlt::malloc(device, dinput_real);
    rlt::malloc(device, gru_output);
    rlt::malloc(device, dloss_dgru_output);
    rlt::malloc(device, grad_W_ir);
    rlt::malloc(device, grad_W_iz);
    rlt::malloc(device, grad_W_in);
    rlt::malloc(device, grad_b_ir);
    rlt::malloc(device, grad_b_iz);
    rlt::malloc(device, grad_b_in);
    rlt::malloc(device, grad_W_hr);
    rlt::malloc(device, grad_W_hz);
    rlt::malloc(device, grad_W_hn);
    rlt::malloc(device, grad_b_hr);
    rlt::malloc(device, grad_b_hz);
    rlt::malloc(device, grad_b_hn);
    rlt::malloc(device, dloss_dgru_output_step);


    rlt::init_weights(device, gru, rng);

    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;
    std::cout << "DATA_FILE_PATH: " << DATA_FILE_PATH << std::endl;
    auto output_file = HighFive::File(std::string(DATA_FILE_PATH), HighFive::File::ReadOnly);
    for(auto epoch_group_name : output_file.listObjectNames()){
        auto epoch_group = output_file.getGroup(epoch_group_name);
        for(auto batch_group_name: epoch_group.listObjectNames()){
            auto batch_group = rlt::get_group(device, epoch_group, batch_group_name);
            rlt::load(device, input, batch_group, "input");
            bool d_input_set = false;
            if(batch_group.group.exist("d_input")){
                rlt::load(device, dinput_real, batch_group, "d_input");
                d_input_set = true;
            }
            rlt::load(device, gru_output, batch_group, "gru_output");
            auto weight_group = rlt::get_group(device, batch_group, "weights");
            using VIEW_SPEC = rlt::tensor::ViewSpec<0, GRU_CONFIG::HIDDEN_DIM>;
            auto W_ir = view_range(device, gru.weights_input.parameters, 0*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto W_iz = view_range(device, gru.weights_input.parameters, 1*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto W_in = view_range(device, gru.weights_input.parameters, 2*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_ir = view_range(device, gru.biases_input.parameters, 0*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_iz = view_range(device, gru.biases_input.parameters, 1*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_in = view_range(device, gru.biases_input.parameters, 2*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            using VIEW_SPEC_DOUBLE = rlt::tensor::ViewSpec<0, 2*GRU_CONFIG::HIDDEN_DIM>;
            auto W_hrz = view_range(device, gru.weights_hidden.parameters, 0*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC_DOUBLE{});
            auto W_hr = view_range(device, gru.weights_hidden.parameters, 0*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto W_hz = view_range(device, gru.weights_hidden.parameters, 1*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto W_hn = view_range(device, gru.weights_hidden.parameters, 2*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_hr = view_range(device, gru.biases_hidden.parameters, 0*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_hz = view_range(device, gru.biases_hidden.parameters, 1*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            auto b_hn = view_range(device, gru.biases_hidden.parameters, 2*GRU_CONFIG::HIDDEN_DIM, VIEW_SPEC{});
            rlt::load(device, W_ir, weight_group, "W_ir");
            rlt::load(device, W_iz, weight_group, "W_iz");
            rlt::load(device, W_in, weight_group, "W_in");
            rlt::load(device, W_hr, weight_group, "W_hr");
            rlt::load(device, W_hz, weight_group, "W_hz");
            rlt::load(device, W_hn, weight_group, "W_hn");
            rlt::load(device, b_ir, weight_group, "b_ir");
            rlt::load(device, b_iz, weight_group, "b_iz");
            rlt::load(device, b_in, weight_group, "b_in");
            rlt::load(device, b_hr, weight_group, "b_hr");
            rlt::load(device, b_hz, weight_group, "b_hz");
            rlt::load(device, b_hn, weight_group, "b_hn");
            rlt::load(device, dloss_dgru_output, batch_group, "d_loss_d_y_pred_gru");
            ASSERT_FALSE(rlt::is_nan(device, gru.weights_input.parameters));
            ASSERT_FALSE(rlt::is_nan(device, gru.weights_hidden.parameters));
            ASSERT_FALSE(rlt::is_nan(device, gru.biases_input.parameters));
            ASSERT_FALSE(rlt::is_nan(device, gru.biases_hidden.parameters));
            ASSERT_FALSE(rlt::is_nan(device, gru.initial_hidden_state.parameters));
            rlt::forward(device, gru, input, buffer, rng);
//            rlt::print(device, gru.output);
            T abs_diff = rlt::abs_diff(device, gru_output, gru.output) / (decltype(gru_output)::SPEC::SIZE);
            ASSERT_LT(abs_diff, 1e-15);
            std::cout << "abs_diff: " << abs_diff << std::endl;
//            auto dloss_dgru_output_view = rlt::view(device, dloss_dgru_output, 0, rlt::tensor::ViewSpec<1>{});
//            rlt::print(device, dloss_dgru_output_view);
            rlt::zero_gradient(device, gru);
            for(TI step=SEQUENCE_LENGTH-1; true; step--){
                auto backward_group = rlt::get_group(device, batch_group, "backward");
                auto gradient_group_step = rlt::get_group(device, backward_group, std::to_string(step));
                rlt::load(device, grad_W_ir, gradient_group_step, "W_ir");
                rlt::load(device, grad_W_iz, gradient_group_step, "W_iz");
                rlt::load(device, grad_W_in, gradient_group_step, "W_in");
                rlt::load(device, grad_W_hr, gradient_group_step, "W_hr");
                rlt::load(device, grad_W_hz, gradient_group_step, "W_hz");
                rlt::load(device, grad_W_hn, gradient_group_step, "W_hn");
                rlt::load(device, grad_b_ir, gradient_group_step, "b_ir");
                rlt::load(device, grad_b_iz, gradient_group_step, "b_iz");
                rlt::load(device, grad_b_in, gradient_group_step, "b_in");
                rlt::load(device, grad_b_hr, gradient_group_step, "b_hr");
                rlt::load(device, grad_b_hz, gradient_group_step, "b_hz");
                rlt::load(device, grad_b_hn, gradient_group_step, "b_hn");


                if constexpr (USE_RESET_MODE){
                    using RESET_MODE_SPEC = rlt::nn::layers::gru::ResetModeSpecification<TI, decltype(reset)>;
                    using RESET_MODE = rlt::nn::layers::gru::ResetMode<rlt::mode::Default<>, RESET_MODE_SPEC>;
                    rlt::Mode<RESET_MODE> reset_mode;
                    reset_mode.reset_container = reset;
                    rlt::set_all(device, reset, false);
                    auto first_step_reset_view = rlt::view(device, reset, 0);
                    rlt::set_all(device, first_step_reset_view, true);
                    rlt::_backward<true, true>(device, gru, input, dloss_dgru_output, dinput, buffer, step, reset_mode);
                }
                else{
                    rlt::_backward<true, true>(device, gru, input, dloss_dgru_output, dinput, buffer, step);
                }


                std::cout << "Step: " << step << std::endl;

                T abs_diff_d_input = 0;
                if(d_input_set){
                    auto d_input_step_view = rlt::view(device, dinput, step);
                    auto d_input_real_step_view = rlt::view(device, dinput_real, step);
                    abs_diff_d_input = rlt::abs_diff(device, d_input_step_view, d_input_real_step_view);
                    std::cout << "abs_diff_d_input: " << abs_diff_d_input << std::endl;
                }

                auto grad_W_hr_view = rlt::view_range(device, gru.weights_hidden.gradient, 0*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_hr = rlt::abs_diff(device, grad_W_hr_view, grad_W_hr)/decltype(grad_W_hr)::SPEC::SIZE;
                std::cout << "abs_diff_W_hr: " << abs_diff_W_hr << std::endl;

                auto grad_b_hr_view = rlt::view_range(device, gru.biases_hidden.gradient, 0*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_hr = rlt::abs_diff(device, grad_b_hr_view, grad_b_hr)/decltype(grad_b_hr)::SPEC::SIZE;
                std::cout << "abs_diff_b_hr: " << abs_diff_b_hr << std::endl;

                auto grad_W_hz_view = rlt::view_range(device, gru.weights_hidden.gradient, 1*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_hz = rlt::abs_diff(device, grad_W_hz_view, grad_W_hz)/decltype(grad_W_hz)::SPEC::SIZE;
                std::cout << "abs_diff_W_hz: " << abs_diff_W_hz << std::endl;

                auto grad_b_hz_view = rlt::view_range(device, gru.biases_hidden.gradient, 1*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_hz = rlt::abs_diff(device, grad_b_hz_view, grad_b_hz)/decltype(grad_b_hz)::SPEC::SIZE;
                std::cout << "abs_diff_b_hz: " << abs_diff_b_hz << std::endl;

                auto grad_W_hn_view = rlt::view_range(device, gru.weights_hidden.gradient, 2*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_hn = rlt::abs_diff(device, grad_W_hn_view, grad_W_hn)/decltype(grad_W_hn)::SPEC::SIZE;
                std::cout << "abs_diff_W_hn: " << abs_diff_W_hn << std::endl;

                auto grad_b_hn_view = rlt::view_range(device, gru.biases_hidden.gradient, 2*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_hn = rlt::abs_diff(device, grad_b_hn_view, grad_b_hn)/decltype(grad_b_hn)::SPEC::SIZE;
                std::cout << "abs_diff_b_hn: " << abs_diff_b_hn << std::endl;

                auto grad_W_ir_view = rlt::view_range(device, gru.weights_input.gradient, 0*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_ir = rlt::abs_diff(device, grad_W_ir_view, grad_W_ir)/decltype(grad_W_ir)::SPEC::SIZE;
                std::cout << "abs_diff_W_ir: " << abs_diff_W_ir << std::endl;

                auto grad_b_ir_view = rlt::view_range(device, gru.biases_input.gradient, 0*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_ir = rlt::abs_diff(device, grad_b_ir_view, grad_b_hr)/decltype(grad_b_ir)::SPEC::SIZE;
                std::cout << "abs_diff_b_ir: " << abs_diff_b_ir << std::endl;

                auto grad_W_iz_view = rlt::view_range(device, gru.weights_input.gradient, 1*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_iz = rlt::abs_diff(device, grad_W_iz_view, grad_W_iz)/decltype(grad_W_iz)::SPEC::SIZE;
                std::cout << "abs_diff_W_iz: " << abs_diff_W_iz << std::endl;

                auto grad_b_iz_view = rlt::view_range(device, gru.biases_input.gradient, 1*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_iz = rlt::abs_diff(device, grad_b_iz_view, grad_b_iz)/decltype(grad_b_iz)::SPEC::SIZE;
                std::cout << "abs_diff_b_iz: " << abs_diff_b_iz << std::endl;

                auto grad_W_in_view = rlt::view_range(device, gru.weights_input.gradient, 2*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_W_in = rlt::abs_diff(device, grad_W_in_view, grad_W_in)/decltype(grad_W_in)::SPEC::SIZE;
                std::cout << "abs_diff_W_in: " << abs_diff_W_in << std::endl;

                auto grad_b_in_view = rlt::view_range(device, gru.biases_input.gradient, 2*HIDDEN_DIM, rlt::tensor::ViewSpec<0, HIDDEN_DIM>{});
                T abs_diff_b_in = rlt::abs_diff(device, grad_b_in_view, grad_b_in)/decltype(grad_b_in)::SPEC::SIZE;
                std::cout << "abs_diff_b_in: " << abs_diff_b_in << std::endl;


                if(d_input_set){
                    ASSERT_LT(abs_diff_d_input, EPSILON);
                }
                ASSERT_LT(abs_diff_W_hr, EPSILON);
                ASSERT_LT(abs_diff_b_hr, EPSILON);
                ASSERT_LT(abs_diff_W_hz, EPSILON);
                ASSERT_LT(abs_diff_b_hz, EPSILON);
                ASSERT_LT(abs_diff_W_hn, EPSILON);
                ASSERT_LT(abs_diff_b_hn, EPSILON);

                ASSERT_LT(abs_diff_W_ir, EPSILON);
                ASSERT_LT(abs_diff_b_ir, EPSILON);
                ASSERT_LT(abs_diff_W_iz, EPSILON);
                ASSERT_LT(abs_diff_b_iz, EPSILON);
                ASSERT_LT(abs_diff_W_in, EPSILON);
                ASSERT_LT(abs_diff_b_in, EPSILON);
                if(step == 0){
                    break;
                }
            }

        }
    }
    rlt::free(device, gru);
    rlt::free(device, buffer);
    rlt::free(device, reset);
    rlt::free(device, input);
    rlt::free(device, dinput);
    rlt::free(device, dinput_real);
    rlt::free(device, gru_output);
    rlt::free(device, dloss_dgru_output);
    rlt::free(device, grad_W_ir);
    rlt::free(device, grad_W_iz);
    rlt::free(device, grad_W_in);
    rlt::free(device, grad_b_ir);
    rlt::free(device, grad_b_iz);
    rlt::free(device, grad_b_in);
    rlt::free(device, grad_W_hr);
    rlt::free(device, grad_W_hz);
    rlt::free(device, grad_W_hn);
    rlt::free(device, grad_b_hr);
    rlt::free(device, grad_b_hz);
    rlt::free(device, grad_b_hn);
    rlt::free(device, dloss_dgru_output_step);
}

TEST(RL_TOOLS_NN_LAYERS_GRU, LOAD_GRU_TORCH_DEFAULT_MODE){
    std::string DATA_FILE_NAME = "gru_training_trace.h5";
    test_loading<false>(DATA_FILE_NAME);
}
TEST(RL_TOOLS_NN_LAYERS_GRU, LOAD_GRU_TORCH_RESET_MODE){
    std::string DATA_FILE_NAME = "gru_training_trace.h5";
    test_loading<true>(DATA_FILE_NAME);
}

TEST(RL_TOOLS_NN_LAYERS_GRU, LOAD_GRU_JAX_DEFAULT_MODE){
    std::string DATA_FILE_NAME = "gru_training_trace_jax.h5";
    test_loading<false>(DATA_FILE_NAME);
}
TEST(RL_TOOLS_NN_LAYERS_GRU, LOAD_GRU_JAX_RESET_MODE){
    std::string DATA_FILE_NAME = "gru_training_trace_jax.h5";
    test_loading<true>(DATA_FILE_NAME);
}

