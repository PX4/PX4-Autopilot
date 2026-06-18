#include <rl_tools/operations/cpu.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/utils/generic/memcpy.h>
#include "../utils/utils.h"
#include <sstream>
#include <random>
#include <iostream>
#include <vector>
#include <filesystem>
#include <gtest/gtest.h>
#include <highfive/H5File.hpp>

#include "default_network_mlp.h"
#include <rl_tools/nn_models/persist.h>
//#define SKIP_TESTS
//#define SKIP_BACKPROP_TESTS
//#define SKIP_ADAM_TESTS
//#define SKIP_OVERFITTING_TESTS
//#define SKIP_TRAINING_TESTS


using DEVICE = NN_DEVICE;
using TI = typename DEVICE::index_t;

constexpr TI N_WEIGHTS = ((INPUT_DIM + 1) * LAYER_1_DIM + (LAYER_1_DIM + 1) * LAYER_2_DIM + (LAYER_2_DIM + 1) * OUTPUT_DIM);

using NetworkType_1 = NetworkType;

template <typename DEVICE, typename T, typename NT>
T abs_diff_network(DEVICE& device, const NT network, const HighFive::Group g){
    T acc = 0;
    std::vector<std::vector<T>> weights;
    g.getDataSet("input_layer/weight").read(weights);
    acc += abs_diff_matrix(matrix_view(device, network.input_layer.weights.parameters), weights);
    return acc;
}

//template <typename DEVICE, typename SPEC>
//typename SPEC::T abs_diff_network(const rlt::nn_models::three_layer_fc::NeuralNetwork<DEVICE, SPEC> network, const HighFive::Group g){
//    using T = typename SPEC::T;
//    T acc = 0;
//    std::vector<std::vector<T>> weights;
//    g.getDataSet("input_layer/weight").read(weights);
//    acc += abs_diff_matrix<T, LAYER_1_DIM, INPUT_DIM>(network.input_layer.weights, weights);
//    return acc;
//}

template <typename NetworkType>
class NeuralNetworkTestLoadWeights : public NeuralNetworkTest {
protected:
    NeuralNetworkTestLoadWeights(){
        model_name = "model_1";
        rlt::malloc(device, network);
        rlt::malloc(device, network_buffers);
        auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
        data_file.getDataSet("model_1/gradients/0/input_layer/weight").read(batch_0_input_layer_weights_grad);
        data_file.getDataSet("model_1/gradients/0/input_layer/bias").read(batch_0_input_layer_biases_grad);
        data_file.getDataSet("model_1/gradients/0/hidden_layers/0/weight").read(batch_0_hidden_layer_0_weights_grad);
        data_file.getDataSet("model_1/gradients/0/hidden_layers/0/bias").read(batch_0_hidden_layer_0_biases_grad);
        data_file.getDataSet("model_1/gradients/0/output_layer/weight").read(batch_0_output_layer_weights_grad);
        data_file.getDataSet("model_1/gradients/0/output_layer/bias").read(batch_0_output_layer_biases_grad);
        this->reset();
        DTYPE input[INPUT_DIM];
        DTYPE output[OUTPUT_DIM];
        standardise<DTYPE, TI, INPUT_DIM>(X_train[0].data(), X_mean.data(), X_std.data(), input);
        standardise<DTYPE, TI, OUTPUT_DIM>(Y_train[0].data(), Y_mean.data(), Y_std.data(), output);
        rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
        input_matrix._data = input;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
        output_matrix._data = output;
        bool rng = false;
        rlt::forward(device, network, input_matrix, network_buffers, rng);
//        rlt::forward(device, network, input);
        DTYPE d_loss_d_output[OUTPUT_DIM];
        rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
        d_loss_d_output_matrix._data = d_loss_d_output;
        rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix);
//        rlt::nn::loss_functions::d_mse_d_x<NN_DEVICE, DTYPE, OUTPUT_DIM, 1>(device, network.output_layer.output.data, output, d_loss_d_output);
        DTYPE d_input[INPUT_DIM];
        rlt::zero_gradient(device, network);
//        rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix = {d_loss_d_output};
        rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
        d_input_matrix._data = d_input;
        rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
//        rlt::backward(device, network, input, d_loss_d_output, d_input);
    }
    void reset(){

        auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
        data_file.getDataSet(model_name + "/init/input_layer/weight").read(input_layer_weights);
        data_file.getDataSet(model_name + "/init/input_layer/bias").read(input_layer_biases);
        data_file.getDataSet(model_name + "/init/hidden_layers/0/weight").read(hidden_layer_0_weights);
        data_file.getDataSet(model_name + "/init/hidden_layers/0/bias").read(hidden_layer_0_biases);
        data_file.getDataSet(model_name + "/init/output_layer/weight").read(output_layer_weights);
        data_file.getDataSet(model_name + "/init/output_layer/bias").read(output_layer_biases);
        auto model_group = rlt::get_group(device, data_file, model_name);
        auto init_group = rlt::get_group(device, model_group, "init");
        auto input_layer_group = rlt::get_group(device, init_group, "input_layer");
        auto hidden_layer_group = rlt::get_group(device, init_group, "hidden_layers");
        auto hidden_layer_0_group = rlt::get_group(device, hidden_layer_group, "0");
        auto output_layer_group = rlt::get_group(device, init_group, "output_layer");
        rlt::load(device, network.input_layer.weights.parameters, input_layer_group, "weight");
        rlt::load(device, network.input_layer.biases.parameters, input_layer_group, "bias");
        rlt::load(device, network.hidden_layers[0].weights.parameters, hidden_layer_0_group, "weight");
        rlt::load(device, network.hidden_layers[0].biases.parameters, hidden_layer_0_group, "bias");
        rlt::load(device, network.output_layer.weights.parameters, output_layer_group, "weight");
        rlt::load(device, network.output_layer.biases.parameters, output_layer_group, "bias");
    }

    NN_DEVICE device;
    NetworkType network;
    typename NetworkType::template Buffer<BATCH_SIZE> network_buffers;
    std::vector<std::vector<DTYPE>> input_layer_weights;
    std::vector<DTYPE> input_layer_biases;
    std::vector<std::vector<DTYPE>> hidden_layer_0_weights;
    std::vector<DTYPE> hidden_layer_0_biases;
    std::vector<std::vector<DTYPE>> output_layer_weights;
    std::vector<DTYPE> output_layer_biases;
    std::vector<std::vector<DTYPE>> batch_0_input_layer_weights_grad;
    std::vector<DTYPE> batch_0_input_layer_biases_grad;
    std::vector<std::vector<DTYPE>> batch_0_hidden_layer_0_weights_grad;
    std::vector<DTYPE> batch_0_hidden_layer_0_biases_grad;
    std::vector<std::vector<DTYPE>> batch_0_output_layer_weights_grad;
    std::vector<DTYPE> batch_0_output_layer_biases_grad;
};

constexpr DTYPE BACKWARD_PASS_GRADIENT_TOLERANCE (1e-8);
#ifndef SKIP_BACKPROP_TESTS
using RL_TOOLS_NN_MLP_BACKWARD_PASS = NeuralNetworkTestLoadWeights<NetworkType_1>;
#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, input_layer_weights) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.input_layer.weights.gradient), batch_0_input_layer_weights_grad);
    std::cout << "input_layer_weights diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * LAYER_1_DIM * INPUT_DIM);
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, input_layer_biases) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.input_layer.biases.gradient), batch_0_input_layer_biases_grad.data());
    std::cout << "input_layer_biases diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * LAYER_1_DIM);
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, hidden_layer_0_weights) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.hidden_layers[0].weights.gradient), batch_0_hidden_layer_0_weights_grad);
    std::cout << "hidden_layer_0_weights diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * LAYER_2_DIM * LAYER_1_DIM);
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, hidden_layer_0_biases) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.hidden_layers[0].biases.gradient), batch_0_hidden_layer_0_biases_grad.data());
    std::cout << "hidden_layer_0_biases diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * LAYER_2_DIM);
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, output_layer_weights) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.output_layer.weights.gradient), batch_0_output_layer_weights_grad);
    std::cout << "output_layer_weights diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * OUTPUT_DIM * LAYER_2_DIM);
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_BACKWARD_PASS, output_layer_biases) {
    DTYPE out = abs_diff_matrix(matrix_view(device, network.output_layer.biases.gradient), batch_0_output_layer_biases_grad.data());
    std::cout << "output_layer_biases diff: " << out << std::endl;
    ASSERT_LT(out, BACKWARD_PASS_GRADIENT_TOLERANCE * OUTPUT_DIM);
}
#endif
#endif


#ifndef SKIP_ADAM_TESTS
typedef RL_TOOLS_NN_MLP_BACKWARD_PASS RL_TOOLS_NN_MLP_ADAM_UPDATE;
#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_ADAM_UPDATE, AdamUpdate) {
    this->reset();
    rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>> optimizer;
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    rlt::get_ref(device, optimizer.parameters, 0).epsilon_sqrt = 0;
//    optimizer.parameters = rlt::nn::optimizers::adam::default_parameters_tensorflow<DTYPE>;
    using TI = typename DEVICE::index_t;

    auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
    std::vector<std::vector<DTYPE>> batch_0_input_layer_weights;
    std::vector<DTYPE> batch_0_input_layer_biases;
    std::vector<std::vector<DTYPE>> batch_0_hidden_layer_0_weights;
    std::vector<DTYPE> batch_0_hidden_layer_0_biases;
    std::vector<std::vector<DTYPE>> batch_0_output_layer_weights;
    std::vector<DTYPE> batch_0_output_layer_biases;
    data_file.getDataSet("model_1/weights/0/input_layer/weight").read(batch_0_input_layer_weights);
    data_file.getDataSet("model_1/weights/0/input_layer/bias").read(batch_0_input_layer_biases);
    data_file.getDataSet("model_1/weights/0/hidden_layers/0/weight").read(batch_0_hidden_layer_0_weights);
    data_file.getDataSet("model_1/weights/0/hidden_layers/0/bias").read(batch_0_hidden_layer_0_biases);
    data_file.getDataSet("model_1/weights/0/output_layer/weight").read(batch_0_output_layer_weights);
    data_file.getDataSet("model_1/weights/0/output_layer/bias").read(batch_0_output_layer_biases);
    DTYPE input[INPUT_DIM];
    DTYPE output[OUTPUT_DIM];
    standardise<DTYPE, TI, INPUT_DIM>(&X_train[0][0], &X_mean[0], &X_std[0], input);
    standardise<DTYPE, TI, OUTPUT_DIM>(&Y_train[0][0], &Y_mean[0], &Y_std[0], output);
    rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
    input_matrix._data = input;
    rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
    output_matrix._data = output;
    bool rng = false;
    rlt::forward(device, network, input_matrix, network_buffers, rng);
    DTYPE d_loss_d_output[OUTPUT_DIM];
    rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
    d_loss_d_output_matrix._data = d_loss_d_output;
    rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix);
    DTYPE d_input[INPUT_DIM];
    rlt::zero_gradient(device, network);
    rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
    d_input_matrix._data = d_input;
    rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
    rlt::reset_optimizer_state(device, optimizer, network);
    rlt::step(device, optimizer, network);

    DTYPE out = abs_diff_matrix(matrix_view(device, network.input_layer.weights.parameters), batch_0_input_layer_weights);
    ASSERT_LT(out, 1.5e-7);
}
#endif
#endif

//#ifdef SKIP_TESTS
//TEST_F(NeuralNetworkTest, OverfitSample) {
//    this->reset();
//
//    DTYPE input[INPUT_DIM];
//    DTYPE output[OUTPUT_DIM];
//    standardise<DTYPE, INPUT_DIM>(X_train[1].data(), X_mean.data(), X_std.data(), input);
//    standardise<DTYPE, OUTPUT_DIM>(Y_train[1].data(), Y_mean.data(), Y_std.data(), output);
//    constexpr TI n_iter = 1000;
//    DTYPE loss = 0;
//    reset_optimizer_state(network);
//    for (TI batch_i = 0; batch_i < n_iter; batch_i++){
//        forward(network, input);
//        DTYPE d_loss_d_output[OUTPUT_DIM];
//        d_mse_d_x<DTYPE, OUTPUT_DIM>(network.output_layer.output, output, d_loss_d_output);
//        loss = mse<DTYPE, OUTPUT_DIM>(network.output_layer.output, output);
//        std::cout << "batch_i: " << batch_i << " loss: " << loss << std::endl;
//
//        zero_gradient(network);
//        DTYPE d_input[INPUT_DIM];
//        backward(network, input, d_loss_d_output, d_input);
//
//        update(network, batch_i + 1, 1);
//    }
//    ASSERT_LT(loss, 5e-10);
//
//
//}
//#endif

#ifndef SKIP_OVERFITTING_TESTS
class RL_TOOLS_NN_MLP_OVERFIT_BATCH : public RL_TOOLS_NN_MLP_BACKWARD_PASS {
public:
    RL_TOOLS_NN_MLP_OVERFIT_BATCH() : RL_TOOLS_NN_MLP_BACKWARD_PASS(){
        model_name = "model_2";
    }
protected:

    void SetUp() override {
        NeuralNetworkTest::SetUp();
        this->reset();
    }
};
#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_OVERFIT_BATCH, OverfitBatch) {
    this->reset();
    rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>> optimizer;
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    rlt::get_ref(device, optimizer.parameters, 0).epsilon_sqrt = 0;

    auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
    HighFive::Group g = data_file.getGroup("model_2/overfit_small_batch");

    using TI = typename DEVICE::index_t;

    constexpr TI n_iter = 1000;
    constexpr TI batch_size = 32;
    DTYPE loss = 0;
    rlt::reset_optimizer_state(device, optimizer, network);
    {
        DTYPE diff = abs_diff_network<DEVICE, DTYPE>(device, network, data_file.getGroup(model_name+"/init"));
        std::cout << "initial diff: " << diff << std::endl;
        ASSERT_EQ(diff, 0);
    }
    for (TI batch_i=0; batch_i < n_iter; batch_i++){
        TI batch_i_real = 0;
        loss = 0;
        rlt::zero_gradient(device, network);
        for (TI sample_i=0; sample_i < batch_size; sample_i++){
            DTYPE input[INPUT_DIM];
            DTYPE output[OUTPUT_DIM];
            standardise<DTYPE, TI, INPUT_DIM>(X_train[batch_i_real * batch_size + sample_i].data(), X_mean.data(), X_std.data(), input);
            standardise<DTYPE, TI,OUTPUT_DIM>(Y_train[batch_i_real * batch_size + sample_i].data(), Y_mean.data(), Y_std.data(), output);
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
            input_matrix._data = input;
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
            output_matrix._data = output;
            bool rng = false;
            rlt::forward(device, network, input_matrix, network_buffers, rng);
            DTYPE d_loss_d_output[OUTPUT_DIM];
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
            d_loss_d_output_matrix._data = d_loss_d_output;
            rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix, DTYPE(1)/batch_size);
            loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);

            DTYPE d_input[INPUT_DIM];
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
            d_input_matrix._data = d_input;
            rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
        }
        loss /= batch_size;

        std::cout << "batch_i " << batch_i << " loss: " << loss << std::endl;

        rlt::step(device, optimizer, network);
//        constexpr TI comp_batch = 100;
//        if(batch_i == comp_batch){
        std::stringstream ss;
        ss << "model_2/overfit_small_batch/" << batch_i;
        DTYPE diff = abs_diff_network<DEVICE, DTYPE>(device, network, data_file.getGroup(ss.str()));
        std::cout << "batch_i: " << batch_i << " diff: " << diff << std::endl;
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
        if (batch_i >= 10){
            break;
        }
#else
        if(batch_i == 10){
            ASSERT_LT(diff, 2.5e-7 * 3 * N_WEIGHTS);
        } else {
            if(batch_i == 100){
                ASSERT_LT(diff, 1.0e-4 * N_WEIGHTS);
            }
        }
#endif
    }
#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_LT(loss, 1e-10);
#endif
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_OVERFIT_BATCH, OverfitBatches) {
    std::vector<DTYPE> losses;
    rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>> optimizer;
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    constexpr TI n_batches = 10;
    for(TI batch_i_real=0; batch_i_real < n_batches; batch_i_real++){
        this->reset();

        constexpr TI n_iter = 1000;
        constexpr TI batch_size = 32;
        DTYPE loss = 0;
        rlt::reset_optimizer_state(device, optimizer, network);
        for (TI batch_i=0; batch_i < n_iter; batch_i++){
            loss = 0;
            rlt::zero_gradient(device, network);
            for (TI sample_i=0; sample_i < batch_size; sample_i++){
                DTYPE input[INPUT_DIM];
                DTYPE output[OUTPUT_DIM];
                standardise<DTYPE, TI, INPUT_DIM>(X_train[batch_i_real * batch_size + sample_i].data(), X_mean.data(), X_std.data(), input);
                standardise<DTYPE, TI,OUTPUT_DIM>(Y_train[batch_i_real * batch_size + sample_i].data(), Y_mean.data(), Y_std.data(), output);
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
                input_matrix._data = input;
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
                output_matrix._data = output;
                bool rng = false;
                rlt::forward(device, network, input_matrix, network_buffers, rng);
                DTYPE d_loss_d_output[OUTPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
                d_loss_d_output_matrix._data = d_loss_d_output;
                rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix, DTYPE(1)/batch_size);
                loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);

                DTYPE d_input[INPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
                d_input_matrix._data = d_input;
                rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
//                rlt::backward(device, network, input, d_loss_d_output, d_input);
            }
            loss /= batch_size;

//            std::cout << "batch_i " << batch_i << " loss: " << loss << std::endl;

            rlt::step(device, optimizer, network);
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (batch_i >= 10){
                break;
            }
#endif
        }
        std::cout << "batch_i_real " << batch_i_real << " loss: " << loss << std::endl;
        losses.push_back(loss);
    }
    DTYPE mean_loss = accumulate(losses.begin(), losses.end(), (DTYPE)0) / n_batches;
    DTYPE std_loss = 0;
    for(auto loss : losses){
        std_loss += (loss - mean_loss) * (loss - mean_loss);
    }
    std_loss = sqrt(std_loss / n_batches);
    DTYPE min_loss = *min_element(losses.begin(), losses.end());
    DTYPE max_loss = *max_element(losses.begin(), losses.end());
    std::sort(losses.begin(), losses.end());
    DTYPE perc1_loss = losses[(TI)(n_batches * 0.01)];
    DTYPE perc5_loss = losses[(TI)(n_batches * 0.05)];
    DTYPE perc95_loss = losses[(TI)(n_batches * 0.95)];
    DTYPE perc99_loss = losses[(TI)(n_batches * 0.99)];

    constexpr DTYPE mean_loss_target   = 3.307228189225464e-12;
    constexpr DTYPE std_loss_target    = 1.363137554222238e-11;
    constexpr DTYPE min_loss_target    = 1.540076829357074e-14;
    constexpr DTYPE max_loss_target    = 1.1320114290391814e-10;
    constexpr DTYPE perc1_loss_target  = 2.662835865368629e-14;
    constexpr DTYPE perc5_loss_target  = 3.350572399960167e-14;
    constexpr DTYPE prec95_loss_target = 1.266289100486372e-11;
    constexpr DTYPE prec99_loss_target = 7.271057700375415e-11;
    constexpr DTYPE assertion_approx_factor = 2;
    std::cout << "mean_loss "   << mean_loss   << " mean_loss_target: " << mean_loss_target << std::endl;
    std::cout << "std_loss "    << std_loss    << " std_loss_target: " << std_loss_target << std::endl;
    std::cout << "min_loss "    << min_loss    << " min_loss_target: " << min_loss_target << std::endl;
    std::cout << "max_loss "    << max_loss    << " max_loss_target: " << max_loss_target << std::endl;
    std::cout << "perc1_loss "  << perc1_loss  << " perc1_loss_target: " << perc1_loss_target << std::endl;
    std::cout << "perc5_loss "  << perc5_loss  << " perc5_loss_target: " << perc5_loss_target << std::endl;
    std::cout << "perc95_loss " << perc95_loss << " prec95_loss_target: " << prec95_loss_target << std::endl;
    std::cout << "perc99_loss " << perc99_loss << " prec99_loss_target: " << prec99_loss_target << std::endl;

    ASSERT_LT(mean_loss_target, 3.307228189225464e-12 * assertion_approx_factor);
    ASSERT_LT(std_loss_target, 1.363137554222238e-11 * assertion_approx_factor);
    ASSERT_LT(min_loss_target, 1.540076829357074e-14 * assertion_approx_factor);
    ASSERT_LT(max_loss_target, 1.1320114290391814e-10 * assertion_approx_factor);
    ASSERT_LT(perc1_loss_target, 2.662835865368629e-14 * assertion_approx_factor);
    ASSERT_LT(perc5_loss_target, 3.350572399960167e-14 * assertion_approx_factor);
    ASSERT_LT(prec95_loss_target, 1.266289100486372e-11 * assertion_approx_factor);
    ASSERT_LT(prec99_loss_target, 7.271057700375415e-11 * assertion_approx_factor);
}
#endif
#endif

class RL_TOOLS_NN_MLP_TRAIN_MODEL : public NeuralNetworkTestLoadWeights<NetworkType_3> {
public:
    typedef NetworkType_3 NETWORK_TYPE;
    RL_TOOLS_NN_MLP_TRAIN_MODEL() : NeuralNetworkTestLoadWeights<NetworkType_3>(){
        model_name = "model_3";
    }
protected:

    void SetUp() override {
        NeuralNetworkTest::SetUp();
        this->reset();
    }
};
#ifndef SKIP_TRAINING_TESTS
#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_TRAIN_MODEL, TrainModel) {
    rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>> optimizer;
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    std::vector<DTYPE> losses;
    std::vector<DTYPE> val_losses;
    constexpr TI n_epochs = 3;
    this->reset();
    rlt::reset_optimizer_state(device, optimizer, network);
    constexpr TI batch_size = 32;
    TI n_iter = X_train.size() / batch_size;

    for(TI epoch_i=0; epoch_i < n_epochs; epoch_i++){
        DTYPE epoch_loss = 0;
        for (TI batch_i=0; batch_i < n_iter; batch_i++){
            DTYPE loss = 0;
            rlt::zero_gradient(device, network);
            for (TI sample_i=0; sample_i < batch_size; sample_i++){
                DTYPE input[INPUT_DIM];
                DTYPE output[OUTPUT_DIM];
                standardise<DTYPE, TI, INPUT_DIM>(X_train[batch_i * batch_size + sample_i].data(), X_mean.data(), X_std.data(), input);
                standardise<DTYPE, TI,OUTPUT_DIM>(Y_train[batch_i * batch_size + sample_i].data(), Y_mean.data(), Y_std.data(), output);
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
                input_matrix._data = input;
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
                output_matrix._data = output;
                bool rng = false;
                rlt::forward(device, network, input_matrix, network_buffers, rng);
                DTYPE d_loss_d_output[OUTPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
                d_loss_d_output_matrix._data = d_loss_d_output;
                rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix, DTYPE(1)/batch_size);
                loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);

                DTYPE d_input[INPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
                d_input_matrix._data = d_input;
                rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
//                rlt::backward(device, network, input, d_loss_d_output, d_input);
            }
            loss /= batch_size;
            epoch_loss += loss;

//            std::cout << "batch_i " << batch_i << " loss: " << loss << std::endl;

            rlt::step(device, optimizer, network);
            std::cout << "epoch_i " << epoch_i << " batch_i " << batch_i << " loss: " << loss << std::endl;
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (batch_i >= 10){
                break;
            }
#endif
        }
        epoch_loss /= n_iter;
        losses.push_back(epoch_loss);

        DTYPE val_loss = 0;
        for (typename DEVICE::index_t sample_i=0; sample_i < X_val.size(); sample_i++){
            DTYPE input[INPUT_DIM];
            DTYPE output[OUTPUT_DIM];
            standardise<DTYPE, TI, INPUT_DIM>(X_val[sample_i].data(), X_mean.data(), X_std.data(), input);
            standardise<DTYPE, TI,OUTPUT_DIM>(Y_val[sample_i].data(), Y_mean.data(), Y_std.data(), output);
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
            input_matrix._data = input;
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
            output_matrix._data = output;
            bool rng = false;
            rlt::forward(device, network, input_matrix, network_buffers, rng);
            val_loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (sample_i >= 10){
                break;
            }
#endif
        }
        val_loss /= X_val.size();
        val_losses.push_back(val_loss);
    }


    for (typename DEVICE::index_t i=0; i < losses.size(); i++){
        std::cout << "epoch_i " << i << " loss: train:" << losses[i] << " val: " << val_losses[i] << std::endl;
    }
    // loss
    // array([0.05651794, 0.02564381, 0.02268129, 0.02161846, 0.02045725,
    //       0.01928116, 0.01860152, 0.01789362, 0.01730141, 0.01681832],
    //      dtype=float32)

    // val_loss
    // array([0.02865824, 0.02282167, 0.02195382, 0.02137529, 0.02023922,
    //       0.0191351 , 0.01818279, 0.01745798, 0.01671058, 0.01628938],
    //      dtype=float32)

#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_LT(losses[0], 0.06);
    ASSERT_LT(losses[1], 0.03);
    ASSERT_LT(losses[2], 0.025);
//    ASSERT_LT(losses[9], 0.02);
    ASSERT_LT(val_losses[0], 0.04);
    ASSERT_LT(val_losses[1], 0.03);
    ASSERT_LT(val_losses[2], 0.025);
#endif
//    ASSERT_LT(val_losses[9], 0.02);

// GELU PyTorch [0.00456139 0.00306715 0.00215886]
}
#endif

#ifndef SKIP_TESTS
TEST_F(RL_TOOLS_NN_MLP_TRAIN_MODEL, ModelInitTrain) {
    rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>> optimizer;
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    NN_DEVICE device;
    NetworkType network;
    rlt::malloc(device, network);
    std::vector<DTYPE> losses;
    std::vector<DTYPE> val_losses;
    constexpr TI n_epochs = 3;
//    this->reset();
    rlt::reset_optimizer_state(device, optimizer, network);
    NN_DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 2);
    rlt::init_weights(device, network, rng);

    constexpr TI batch_size = 32;
    TI n_iter = X_train.size() / batch_size;

    for(TI epoch_i=0; epoch_i < n_epochs; epoch_i++){
        DTYPE epoch_loss = 0;
        for (TI batch_i=0; batch_i < n_iter; batch_i++){
            DTYPE loss = 0;
            rlt::zero_gradient(device, network);
            for (TI sample_i=0; sample_i < batch_size; sample_i++){
                DTYPE input[INPUT_DIM];
                DTYPE output[OUTPUT_DIM];
                standardise<DTYPE, TI, INPUT_DIM>(X_train[batch_i * batch_size + sample_i].data(), X_mean.data(), X_std.data(), input);
                standardise<DTYPE, TI,OUTPUT_DIM>(Y_train[batch_i * batch_size + sample_i].data(), Y_mean.data(), Y_std.data(), output);
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
                input_matrix._data = input;
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
                output_matrix._data = output;
                rlt::forward(device, network, input_matrix, network_buffers, rng);
//                rlt::forward(device, network, input);
                DTYPE d_loss_d_output[OUTPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
                d_loss_d_output_matrix._data = d_loss_d_output;
                rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix, DTYPE(1)/batch_size);
                loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);

                DTYPE d_input[INPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> d_input_matrix;
                d_input_matrix._data = d_input;
                rlt::backward_full(device, network, input_matrix, d_loss_d_output_matrix, d_input_matrix, network_buffers);
//                rlt::backward(device, network, input, d_loss_d_output, d_input);
            }
            loss /= batch_size;
            epoch_loss += loss;

//            std::cout << "batch_i " << batch_i << " loss: " << loss << std::endl;

            rlt::step(device, optimizer, network);
            std::cout << "epoch_i " << epoch_i << " batch_i " << batch_i << " loss: " << loss << std::endl;
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (batch_i >= 10){
                break;
            }
#endif
        }
        epoch_loss /= n_iter;
        losses.push_back(epoch_loss);

        DTYPE val_loss = 0;
        for (TI sample_i=0; sample_i < X_val.size(); sample_i++){
            DTYPE input[INPUT_DIM];
            DTYPE output[OUTPUT_DIM];
            standardise<DTYPE, TI, INPUT_DIM>(X_val[sample_i].data(), X_mean.data(), X_std.data(), input);
            standardise<DTYPE, TI,OUTPUT_DIM>(Y_val[sample_i].data(), Y_mean.data(), Y_std.data(), output);
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
            input_matrix._data = input;
            rlt::Matrix<rlt::matrix::Specification<DTYPE, NN_DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
            output_matrix._data = output;
            rlt::forward(device, network, input_matrix, network_buffers, rng);
//            rlt::forward(device, network, input);
            val_loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, DTYPE(1)/batch_size);
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (sample_i >= 10){
                break;
            }
#endif
        }
        val_loss /= X_val.size();
        val_losses.push_back(val_loss);
    }


    for (TI i=0; i < losses.size(); i++){
        std::cout << "epoch_i " << i << " loss: train:" << losses[i] << " val: " << val_losses[i] << std::endl;
    }
    // loss
    // array([0.05651794, 0.02564381, 0.02268129, 0.02161846, 0.02045725,
    //       0.01928116, 0.01860152, 0.01789362, 0.01730141, 0.01681832],
    //      dtype=float32)

    // val_loss
    // array([0.02865824, 0.02282167, 0.02195382, 0.02137529, 0.02023922,
    //       0.0191351 , 0.01818279, 0.01745798, 0.01671058, 0.01628938],
    //      dtype=float32)

#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_LT(losses[0], 0.06);
    ASSERT_LT(losses[1], 0.03);
    ASSERT_LT(losses[2], 0.025);
//    ASSERT_LT(losses[9], 0.02);
    ASSERT_LT(val_losses[0], 0.04);
    ASSERT_LT(val_losses[1], 0.03);
    ASSERT_LT(val_losses[2], 0.025);
#endif
//    ASSERT_LT(val_losses[9], 0.02);

// GELU PyTorch [0.00456139 0.00306715 0.00215886]
}
#endif
// after refactoring
//11: epoch_i 0 loss: train:0.00180514 val: 0.000883146
//11: epoch_i 1 loss: train:0.000798555 val: 0.000770896
//11: epoch_i 2 loss: train:0.000730709 val: 0.000740552
#endif


