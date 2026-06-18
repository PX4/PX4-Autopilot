#include <rl_tools/operations/cpu.h>
//#include <rl_tools/operations/dummy.h>


#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include "../utils/utils.h"

#include <gtest/gtest.h>

#include <random>
#include <chrono>
#include <highfive/H5File.hpp>


typedef double T;


using DEVICE = rlt::devices::DefaultCPU;
using TI = typename DEVICE::index_t;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
//template <typename T_T>
//struct StructureSpecification{
//    typedef T_T T;
//    static constexpr typename DEVICE::index_t INPUT_DIM = 17;
//    static constexpr typename DEVICE::index_t OUTPUT_DIM = 13;
//    static constexpr int NUM_LAYERS = 3;
//    static constexpr int HIDDEN_DIM = 50;
//    static constexpr rlt::nn::activation_functions::ActivationFunction HIDDEN_ACTIVATION_FUNCTION = rlt::nn::activation_functions::GELU;
//    static constexpr rlt::nn::activation_functions::ActivationFunction OUTPUT_ACTIVATION_FUNCTION = rlt::nn::activation_functions::IDENTITY;
//};

constexpr TI BATCH_SIZE = 32;
constexpr TI INTERNAL_BATCH_SIZE = 1;
using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, 1, 17>; // actual batch size is 1
using NETWORK_CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, DEVICE::index_t, 13, 3, 50, rlt::nn::activation_functions::GELU, rlt::nn::activation_functions::IDENTITY>;


using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE::index_t>;
using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_PARAMETERS>;
using CAPABILITY_ADAM = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, INTERNAL_BATCH_SIZE>;
using NetworkType = rlt::nn_models::mlp::NeuralNetwork<NETWORK_CONFIG, CAPABILITY_ADAM, INPUT_SHAPE>;

std::vector<std::vector<T>> X_train;
std::vector<std::vector<T>> Y_train;
std::vector<std::vector<T>> X_val;
std::vector<std::vector<T>> Y_val;
std::vector<T> X_mean;
std::vector<T> X_std;
std::vector<T> Y_mean;
std::vector<T> Y_std;

constexpr typename DEVICE::index_t INPUT_DIM = rlt::get_last(typename NetworkType::INPUT_SHAPE{});
constexpr typename DEVICE::index_t OUTPUT_DIM = rlt::get_last(typename NetworkType::OUTPUT_SHAPE{});

TEST(RL_TOOLS_NN_MLP_FULL_TRAINING, FULL_TRAINING) {
    // loading data
    std::string DATA_FILE_NAME = "mlp_data.hdf5";
    const char *data_path_stub = RL_TOOLS_MACRO_TO_STR(RL_TOOLS_TEST_DATA_PATH);
    std::string DATA_FILE_PATH = std::string(data_path_stub) + "/" + DATA_FILE_NAME;

    auto data_file = HighFive::File(DATA_FILE_PATH, HighFive::File::ReadOnly);
    data_file.getDataSet("data/X_train").read(X_train);
    data_file.getDataSet("data/Y_train").read(Y_train);
    data_file.getDataSet("data/X_val").read(X_val);
    data_file.getDataSet("data/Y_val").read(Y_val);
    data_file.getDataSet("data/X_mean").read(X_mean);
    data_file.getDataSet("data/X_std").read(X_std);
    data_file.getDataSet("data/Y_mean").read(Y_mean);
    data_file.getDataSet("data/Y_std").read(Y_std);



    DEVICE device;
    using TI = typename DEVICE::index_t;
    OPTIMIZER optimizer;
    NetworkType network;
    typename NetworkType::Buffer<> buffers;
    rlt::malloc(device, optimizer);
    rlt::malloc(device, network);
    rlt::malloc(device, buffers);
    std::vector<T> losses;
    std::vector<T> val_losses;
    std::vector<T> epoch_durations;
    constexpr TI n_epochs = 3;
    //    this->reset();
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, network);
//    typename DEVICE::index_t rng = 2;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 2);
    rlt::init_weights(device, network, rng);

    TI n_iter = X_train.size() / BATCH_SIZE;

    for(TI epoch_i=0; epoch_i < n_epochs; epoch_i++){
        T epoch_loss = 0;
        auto epoch_start_time = std::chrono::high_resolution_clock::now();
        for (TI batch_i=0; batch_i < n_iter; batch_i++){
            T loss = 0;
            rlt::zero_gradient(device, network);
            for (TI sample_i=0; sample_i < BATCH_SIZE; sample_i++){
                T input[INPUT_DIM];
                T output[OUTPUT_DIM];
                standardise<T, TI, INPUT_DIM>(X_train[batch_i * BATCH_SIZE + sample_i].data(), X_mean.data(), X_std.data(), input);
                standardise<T, TI,OUTPUT_DIM>(Y_train[batch_i * BATCH_SIZE + sample_i].data(), Y_mean.data(), Y_std.data(), output);
                rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
                input_matrix._data = input;
                rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
                output_matrix._data = output;
                bool rng = false;
                rlt::forward(device, network, input_matrix, buffers, rng);
                T d_loss_d_output[OUTPUT_DIM];
                rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, OUTPUT_DIM>> d_loss_d_output_matrix;
                d_loss_d_output_matrix._data = d_loss_d_output;
                rlt::nn::loss_functions::mse::gradient(device, network.output_layer.output, output_matrix, d_loss_d_output_matrix, T(1)/T(BATCH_SIZE));
                loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, T(1)/T(BATCH_SIZE));

                rlt::backward(device, network, input_matrix, d_loss_d_output_matrix, buffers);
            }
            loss /= BATCH_SIZE;
            epoch_loss += loss;

            //            std::cout << "batch_i " << batch_i << " loss: " << loss << std::endl;

            rlt::step(device, optimizer, network);
            if(batch_i % 1000 == 0){
                std::cout << "epoch_i " << epoch_i << " batch_i " << batch_i << " loss: " << loss << std::endl;
            }
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (batch_i >= 10){
                break;
            }
#endif
        }
        // save epoch_duration to epoch_durations
        auto epoch_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<T> epoch_duration = epoch_end_time - epoch_start_time;
        epoch_durations.push_back(epoch_duration.count());

        epoch_loss /= n_iter;
        losses.push_back(epoch_loss);

        T val_loss = 0;
        for (DEVICE::index_t sample_i=0; sample_i < X_val.size(); sample_i++){
            T input[INPUT_DIM];
            T output[OUTPUT_DIM];
            standardise<T, TI, INPUT_DIM>(X_val[sample_i].data(), X_mean.data(), X_std.data(), input);
            standardise<T, TI,OUTPUT_DIM>(Y_val[sample_i].data(), Y_mean.data(), Y_std.data(), output);

            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, INPUT_DIM>> input_matrix;
            input_matrix._data = input;
            rlt::Matrix<rlt::matrix::Specification<T, DEVICE::index_t, 1, OUTPUT_DIM>> output_matrix;
            output_matrix._data = output;
            bool rng = false;
            rlt::forward(device, network, input_matrix, buffers, rng);
            val_loss += rlt::nn::loss_functions::mse::evaluate(device, network.output_layer.output, output_matrix, T(1)/BATCH_SIZE);
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
            if (sample_i >= 10){
                break;
            }
#endif
        }
        val_loss /= X_val.size();
        val_losses.push_back(val_loss);
    }


    // epoch duration should be around 13s (Lenovo P1, Intel(R) Core(TM) i9-10885H CPU @ 2.40GHz) when compiled with -O3
    for (typename DEVICE::index_t i=0; i < losses.size(); i++){
        std::cout << "epoch_i " << i << " loss: train:" << losses[i] << " val: " << val_losses[i] << " duration: " << epoch_durations[i] << std::endl;
    }

#ifndef RL_TOOLS_TESTS_CODE_COVERAGE
    ASSERT_LT(losses[0], 0.005);
    ASSERT_LT(losses[1], 0.002);
    ASSERT_LT(losses[2], 0.002);
    ASSERT_LT(val_losses[0], 0.002);
    ASSERT_LT(val_losses[1], 0.001);
    ASSERT_LT(val_losses[2], 0.001);
#endif

    // GELU PyTorch [0.00456139 0.00306715 0.00215886]

//    After refactoring
//    12: epoch_i 0 loss: train:0.000872663 val: 9.64629e-05
//    12: epoch_i 1 loss: train:6.79054e-05 val: 4.71358e-05
//    12: epoch_i 2 loss: train:4.67507e-05 val: 3.80284e-05


//    TANH
//    epoch_i 0 loss: train:0.00265808 val: 0.00041066
//    epoch_i 1 loss: train:0.000322035 val: 0.000304242
//    epoch_i 2 loss: train:0.000230863 val: 0.000214781
//    RELU
//    epoch_i 0 loss: train:0.00178393 val: 0.000915656
//    epoch_i 1 loss: train:0.000778314 val: 0.000739406
//    epoch_i 2 loss: train:0.000696363 val: 0.000696439
}
