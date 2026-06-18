#define MUX
#ifdef RL_TOOLS_ENABLE_TRACY
#include "Tracy.hpp"
#endif
//#define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#ifdef MUX
#include <rl_tools/operations/cpu_mux.h>
#else
#include <rl_tools/operations/cpu.h>
#endif
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#ifdef MUX
#include <rl_tools/nn/operations_cpu_mux.h>
#else
#include <rl_tools/nn/operations_cpu.h>
#endif
#include <rl_tools/nn/loss_functions/mse/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/embedding/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include "dataset.h"

namespace rlt = rl_tools;

#include <chrono>
#include <queue>
#include <algorithm>
#include <filesystem>

#ifdef RL_TOOLS_ENABLE_JSON
#include <nlohmann/json.hpp>
#endif


#ifdef MUX
using DEVICE = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

template <typename TYPE_POLICY, typename TI>
struct Config{
    using T = typename TYPE_POLICY::DEFAULT;
    struct BASE{
        static constexpr TI BATCH_SIZE = 16;
        static constexpr TI INPUT_DIM = 1;
        static constexpr TI OUTPUT_DIM = 1;
        static constexpr TI HIDDEN_DIM = 32;
        static constexpr TI SEQUENCE_LENGTH = 512;
        static constexpr TI HORIZON = 100;
        static constexpr TI DATASET_SIZE = 100000;
        static constexpr T PROBABILITY = 5 * 1/((T)HORIZON);
    };

    using PARAMS = BASE;
//    using PARAMS = USEFUL;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, PARAMS::SEQUENCE_LENGTH, PARAMS::BATCH_SIZE, PARAMS::INPUT_DIM>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using RESET_SHAPE = rlt::tensor::Shape<TI, PARAMS::SEQUENCE_LENGTH, PARAMS::BATCH_SIZE, 1>;
    using RESET_TYPE = rlt::tensor::Specification<bool, TI, RESET_SHAPE>;
    using GRU_CONFIG = rlt::nn::layers::gru::Configuration<TYPE_POLICY, TI, PARAMS::HIDDEN_DIM, rlt::nn::parameters::Gradient, true>;
    using GRU = rlt::nn::layers::gru::BindConfiguration<GRU_CONFIG>;
    using DENSE_LAYER1_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMS::HIDDEN_DIM, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
    using DENSE_LAYER1 = rlt::nn::layers::dense::BindConfiguration<DENSE_LAYER1_CONFIG>;
    using DENSE_LAYER2_CONFIG = rlt::nn::layers::dense::Configuration<TYPE_POLICY, TI, PARAMS::OUTPUT_DIM, rlt::nn::activation_functions::ActivationFunction::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>, rlt::nn::parameters::groups::Normal>;
    using DENSE_LAYER2 = rlt::nn::layers::dense::BindConfiguration<DENSE_LAYER2_CONFIG>;

    template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
    using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;

    using MODULE_CHAIN = Module<GRU, Module<DENSE_LAYER1, Module<DENSE_LAYER2>>>;
    using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
    using OUTPUT_TARGET_SHAPE = rlt::tensor::Shape<TI, PARAMS::SEQUENCE_LENGTH, PARAMS::BATCH_SIZE, 1>;
    using OUTPUT_TARGET_SPEC = rlt::tensor::Specification<T, TI, OUTPUT_TARGET_SHAPE>;
    struct ADAM_PARAMS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 2e-2;
    };
    using ADAM_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, ADAM_PARAMS>;
    using ADAM = rlt::nn::optimizers::Adam<ADAM_SPEC>;
};

using CONFIG = Config<TYPE_POLICY, TI>;

template <bool RESET, typename DEVICE, typename INPUT, typename OUTPUT, typename RESET_CONTAINER, typename RNG>
void synthetic_sample(DEVICE& device, INPUT& input, OUTPUT& output_target, RESET_CONTAINER& reset, RNG& rng){
    for(TI batch_i = 0; batch_i < CONFIG::PARAMS::BATCH_SIZE; batch_i++){
        std::vector<T> values;
        for(TI sequence_i = 0; sequence_i < CONFIG::PARAMS::SEQUENCE_LENGTH; sequence_i++){
            // sum in window
            bool reset_now = RESET && (rlt::random::uniform_real_distribution(device.random, (T)0, (T)1, rng) < 0.2/((T)CONFIG::PARAMS::HORIZON));
            if(sequence_i == 0 || reset_now){
                values.clear();
                rlt::set(device, reset, true, sequence_i, batch_i, 0);
            }
            else{
                rlt::set(device, reset, false, sequence_i, batch_i, 0);
            }
            T new_value = 0; //rlt::random::normal_distribution::sample(device.random, (T)0, (T)1, rng);
            if(rlt::random::uniform_real_distribution(device.random, (T)0, (T)1, rng) < CONFIG::PARAMS::PROBABILITY){
                new_value = 1;
            }
            values.push_back(new_value);
            while(values.size() > CONFIG::PARAMS::HORIZON){
                values.erase(values.begin());
            }
            T number_of_ones = std::count(values.begin(), values.end(), 1);
            set(device, input, new_value, sequence_i, batch_i, 0);
            set(device, output_target, (T)number_of_ones/(CONFIG::PARAMS::HORIZON * CONFIG::PARAMS::PROBABILITY), sequence_i, batch_i, 0);
        }
    }
}

int main(){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    std::string data_path, file_name;
    std::string dataset_string;

    typename CONFIG::MODEL model;
    typename CONFIG::MODEL::Buffer<> buffer;
    typename CONFIG::ADAM optimizer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<typename CONFIG::RESET_TYPE> reset;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::OUTPUT_SHAPE>> d_output;
    rlt::Tensor<typename CONFIG::OUTPUT_TARGET_SPEC> output_target;
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, reset);
    rlt::malloc(device, d_output);
    rlt::malloc(device, output_target);
    rlt::init_weights(device, model, rng);
    rlt::malloc(device, optimizer);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, model);
    for(TI epoch_i=0; epoch_i < 1000; epoch_i++){
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_print = start_time;
        for(TI sample_i=0; sample_i < CONFIG::PARAMS::DATASET_SIZE; sample_i += CONFIG::PARAMS::BATCH_SIZE){
            synthetic_sample<true>(device, input, output_target, reset, rng);
            using RESET_MODE_SPEC = rlt::nn::layers::gru::ResetModeSpecification<TI, decltype(reset)>;
            using RESET_MODE = rlt::nn::layers::gru::ResetMode<rlt::mode::Default<>, RESET_MODE_SPEC>;
            rlt::Mode<RESET_MODE> reset_mode;
            reset_mode.reset_container = reset;
            rlt::forward(device, model, input, buffer, rng, reset_mode);
            auto output = rlt::output(device, model);
            auto output_matrix_view = rlt::matrix_view(device, output);
            auto output_target_matrix_view = rlt::matrix_view(device, output_target);
            auto d_output_matrix_view = rlt::matrix_view(device, d_output);
            rlt::nn::loss_functions::mse::gradient(device, output_matrix_view, output_target_matrix_view, d_output_matrix_view);
            T elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() / 1000.0;
            T elapsed_print = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_print).count() / 1000.0;
            rlt::zero_gradient(device, model);
            rlt::backward(device, model, input, d_output, buffer, reset_mode);
            rlt::step(device, optimizer, model);

            if(elapsed_print > 2.0 || sample_i % 1000000 == 0){
                synthetic_sample<false>(device, input, output_target, reset, rng);
                rlt::forward(device, model, input, buffer, rng, reset_mode);
#ifdef RL_TOOLS_ENABLE_JSON
                nlohmann::json batch;
#endif
                for(TI sequence_i = 0; sequence_i < CONFIG::PARAMS::SEQUENCE_LENGTH; sequence_i++){
//                    std::cout << "Step: " << sequence_i << std::endl;
                    for(TI sample_i = 0; sample_i < CONFIG::PARAMS::BATCH_SIZE; sample_i++){
//                        std::cout << "Input: " << rlt::get(device, input, sequence_i, sample_i, 0) << " Reset: " << rlt::get(device, reset, sequence_i, sample_i, 0) << " Target: " << rlt::get(device, output_target, sequence_i, sample_i, 0) << " => " << rlt::get(device, output, sequence_i, sample_i, 0) << std::endl;
                        constexpr TI FP_WIDTH = 15;
                        constexpr TI WIDTH = 2;
                        if(sample_i == 0){
                            std::cout << "Input: " << std::setw(WIDTH) << rlt::get(device, input, sequence_i, sample_i, 0)
                                      << " Reset: " << std::setw(WIDTH) << rlt::get(device, reset, sequence_i, sample_i, 0)
                                      << " Target: " << std::setw(WIDTH) << rlt::get(device, output_target, sequence_i, sample_i, 0)
                                      << " => " << std::setw(FP_WIDTH) << rlt::get(device, output, sequence_i, sample_i, 0)
                                      << std::endl;
                        }
#ifdef RL_TOOLS_ENABLE_JSON
                        nlohmann::json sample;
                        sample["input"] = rlt::get(device, input, sequence_i, sample_i, 0);
                        sample["reset"] = rlt::get(device, reset, sequence_i, sample_i, 0);
                        sample["target"] = rlt::get(device, output_target, sequence_i, sample_i, 0);
                        sample["output"] = rlt::get(device, output, sequence_i, sample_i, 0);
                        batch[sample_i][sequence_i] = sample;
#endif
                    }
                }
                T loss = rlt::nn::loss_functions::mse::evaluate(device, output_matrix_view, output_target_matrix_view);
#ifdef RL_TOOLS_ENABLE_JSON
                nlohmann::json data;
                data["meta"]["BATCH_SIZE"] = CONFIG::PARAMS::BATCH_SIZE;
                data["meta"]["SEQUENCE_LENGTH"] = CONFIG::PARAMS::SEQUENCE_LENGTH;
                data["meta"]["HORIZON"] = CONFIG::PARAMS::HORIZON;
                data["meta"]["PROBABILITY"] = CONFIG::PARAMS::PROBABILITY;
                data["meta"]["HIDDEN_DIM"] = CONFIG::PARAMS::HIDDEN_DIM;
                data["meta"]["LOSS"] = loss;
                data["batch"] = batch;
                std::fstream file;
                file.open("gru_training_sequences.json", std::ios::out);
                file << data.dump(4);
                file.close();
#endif
                last_print = std::chrono::high_resolution_clock::now();
                std::cout << "Epoch: " << epoch_i << " Sample: " << sample_i << " Batch: " << sample_i/CONFIG::PARAMS::BATCH_SIZE << " (" << sample_i/CONFIG::PARAMS::BATCH_SIZE/elapsed << " batch/s)" << " Loss: " << loss << std::endl;
            }
        }
    }
    return 0;
}
