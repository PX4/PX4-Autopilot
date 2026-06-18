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
#include <rl_tools/random/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#ifdef MUX
#include <rl_tools/nn/operations_cpu_mux.h>
#else
#include <rl_tools/nn/operations_cpu.h>
#endif
#include <rl_tools/nn/loss_functions/categorical_cross_entropy/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#ifdef RL_TOOLS_ENABLE_HDF5

#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/embedding/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#endif
#include "dataset.h"

namespace rlt = rl_tools;

#include "gru_model.h"


#include <chrono>
#include <filesystem>


#ifdef MUX
using MATH_DEVICE = rlt::devices::math::CPU;
using RANDOM_DEVICE = rlt::devices::random::Generic<rlt::devices::math::CPU>;
using DEVICE_SPEC = rlt::devices::cpu::Specification<MATH_DEVICE, RANDOM_DEVICE, rlt::LOGGER_FACTORY<>>;
using DEVICE = rlt::devices::DEVICE_FACTORY<DEVICE_SPEC>;
//using DEVICE = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;

using CONFIG = Config<TYPE_POLICY, TI, true>;
using PARAMS = typename CONFIG::PARAMS;

constexpr TI N_EPOCH = 1;

template <typename DEVICE, typename RandomIt, typename RNG>
void shuffle(DEVICE& device, RandomIt first, RandomIt last, RNG& rng) {
    using diff_t = typename std::iterator_traits<RandomIt>::difference_type;
    diff_t size = last - first;
    if (size <= 1) {
        return;
    }
    for (diff_t i = size - 1; i > 0; --i) {
        diff_t j = rlt::random::uniform_int_distribution(device.random, (diff_t)0, i, rng);
        std::swap(first[i], first[j]);
    }
}

int main(){
    DEVICE device;
    typename DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device, rng, 0);

    std::vector<std::tuple<std::string, std::string>> dataset;
    constexpr TI DATASET_SIZE = 1000000;
    static_assert(PARAMS::MEM_DELAY > 1);
    std::string padding(PARAMS::MEM_DELAY, '0');
    for(TI offset=0; offset < DATASET_SIZE; offset++){
        std::string target;
        for(TI i=0; i < PARAMS::MEM_SIZE; i++){
            target += rlt::random::uniform_int_distribution(device.random, (TI)0, (TI)1, rng) == 0 ? '2' : '3';
        }
        std::string full_seq = target + padding + '1' + target;
        std::string input = full_seq.substr(0, full_seq.size() - 1);
        std::string output = full_seq.substr(1, full_seq.size() - 1);
        dataset.emplace_back(std::tuple(input, output));
    }
    shuffle(device, dataset.begin(), dataset.end(), rng);
    std::cout << "Dataset size: " << dataset.size() << std::endl;
    std::cout << "Dataset sample: " << std::endl;
    for(TI i=0; i < 10; i++){
        std::cout << std::get<0>(dataset[i]) << " -> " << std::get<1>(dataset[i]) << std::endl;
    }

    typename CONFIG::MODEL model;
    typename CONFIG::MODEL::Buffer<> buffer;
    typename CONFIG::ADAM optimizer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, CONFIG::MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, CONFIG::MODEL::OUTPUT_SHAPE>> d_output;
    rlt::Tensor<typename CONFIG::OUTPUT_TARGET_SPEC> output_target;
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, d_output);
    rlt::malloc(device, output_target);
    rlt::malloc(device, optimizer);
    rlt::init_weights(device, model, rng);
    rlt::init(device, optimizer);
    rlt::reset_optimizer_state(device, optimizer, model);
    std::cout << "INPUT SHAPE";
    rlt::print(device, decltype(input)::SPEC::SHAPE{});
    std::cout << std::endl;
    for(TI epoch_i=0; epoch_i < N_EPOCH; epoch_i++){
        shuffle(device, dataset.begin(), dataset.end(), rng);
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_print = start_time;
        for(TI sample_i=0; sample_i < dataset.size() - PARAMS::BATCH_SIZE; sample_i += PARAMS::BATCH_SIZE){
#ifdef RL_TOOLS_ENABLE_TRACY
            FrameMark;
#endif
            if(sample_i % 10000 == 0){
                //checkpoint
#ifdef RL_TOOLS_ENABLE_HDF5
                std::filesystem::path FILE_PATH = "model_checkpoint.h5";
                {
                    std::cout << "Checkpointing" << std::endl;
                    auto file = HighFive::File(FILE_PATH.string(), HighFive::File::Overwrite);
                    rlt::zero_gradient(device, model);
                    rlt::reset_forward_state(device, model);
                    auto checkpoint_group = rlt::create_group(device, file, "checkpoint");
                    rlt::save(device, model, checkpoint_group);
                }
                if(sample_i == 0 || sample_i == PARAMS::BATCH_SIZE){ // reload check
                    auto file = HighFive::File(FILE_PATH.string(), HighFive::File::ReadOnly);
                    CONFIG::MODEL model_copy;
                    rlt::malloc(device, model_copy);
                    auto checkpoint_group = rlt::get_group(device, file, "checkpoint");
                    rlt::load(device, model_copy, checkpoint_group);
                    T abs_diff = rlt::abs_diff(device, model, model_copy);
                    rlt::utils::assert_exit(device, abs_diff < 1e-6, "Checkpoint failed");
                    rlt::free(device, model_copy);
                }
#endif
            }

            for(TI batch_i = 0; batch_i < PARAMS::BATCH_SIZE; batch_i++){
                for(TI sequence_i = 0; sequence_i < PARAMS::SEQUENCE_LENGTH; sequence_i++){
                    char input_char = std::get<0>(dataset[sample_i + batch_i])[sequence_i];
                    char output_char = std::get<1>(dataset[sample_i + batch_i])[sequence_i];
                    if(input_char < 0) {
                        input_char = '?';
                    }
                    if(output_char < 0) {
                        output_char = '?';
                    }
                    TI input_index = input_char - '0';
                    TI output_index = output_char - '0';
                    rlt::set(device, input, input_index, sequence_i, batch_i, 0);
                    rlt::set(device, output_target, output_index, sequence_i, batch_i, 0);
                }
            }
            {
#ifdef RL_TOOLS_ENABLE_TRACY
                ZoneScopedN("forward");
#endif
                rlt::forward(device, model, input, buffer, rng);
            }
            auto output_logits = rlt::output(device, model);
            auto output_logits_matrix_view = rlt::matrix_view(device, output_logits);
            auto output_target_matrix_view = rlt::matrix_view(device, output_target);
            auto d_output_matrix_view = rlt::matrix_view(device, d_output);
            {
#ifdef RL_TOOLS_ENABLE_TRACY
                ZoneScopedN("loss_gradient");
#endif
                rlt::nn::loss_functions::categorical_cross_entropy::gradient_tiled(device, output_logits_matrix_view, output_target_matrix_view, d_output_matrix_view);
            }
            T num_correct = 0;
            T num_considered = 0;
            for (TI sequence_i=0; sequence_i < PARAMS::SEQUENCE_LENGTH; sequence_i++){
                for (TI batch_i=0; batch_i < PARAMS::BATCH_SIZE; batch_i++){
                    if (sequence_i < PARAMS::MEM_SIZE + PARAMS::MEM_DELAY){
                        for (TI char_i=0; char_i < PARAMS::NUM_CLASSES; char_i++){
                            rlt::set(device, d_output, 0, sequence_i, batch_i, char_i);
                        }
                    }
                    else{
                        for (TI char_i=0; char_i < PARAMS::NUM_CLASSES; char_i++){
                            T d_output_value = rlt::get(device, d_output, sequence_i, batch_i, char_i);
                            rlt::set(device, d_output, d_output_value * PARAMS::SEQUENCE_LENGTH / (T)PARAMS::MEM_SIZE, sequence_i, batch_i, char_i);
                        }
                    }
                    if (sample_i % 10000 == 0){
                        T max_logit = 0;
                        bool max_logit_set = false;
                        TI max_logit_index = 0;
                        for (TI char_i=0; char_i < PARAMS::NUM_CLASSES; char_i++){
                            T logit = rlt::get(device, output_logits, sequence_i, batch_i, char_i);
                            if (!max_logit_set || logit > max_logit){
                                max_logit = logit;
                                max_logit_set = true;
                                max_logit_index = char_i;
                            }
                        }
                        char output_char_pred = max_logit_index + '0';
                        TI input_index = rlt::get(device, input, sequence_i, batch_i, 0);
                        TI output_index = rlt::get(device, output_target, sequence_i, batch_i, 0);
                        char input_char = input_index + '0';
                        char output_char = output_index + '0';
                        if (sequence_i >= PARAMS::MEM_SIZE + PARAMS::MEM_DELAY){
                            bool correct = output_index == max_logit_index;
                            num_correct += correct;
                            num_considered++;
                            std::cout << input_char << " -> " << output_char << " (" << output_char_pred << ")" << (correct ? "✅" : "❌") << " logit " << max_logit << std::endl;
                        }
                    }
                }
            }
            if (num_considered > 0) {
                std::cout << "Accuracy: " << num_correct / num_considered << std::endl;
            }
            T elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() / 1000.0;
            T elapsed_print = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_print).count() / 1000.0;
            if(sample_i % 100 == 0){
            // if(elapsed_print > 0.2 || sample_i % 10000 == 0){
                auto relevant_output_logits_matrix_view = rlt::view(device, output_logits_matrix_view, rlt::matrix::ViewSpec<PARAMS::MEM_SIZE * PARAMS::BATCH_SIZE, PARAMS::NUM_CLASSES>{}, (PARAMS::MEM_SIZE + PARAMS::MEM_DELAY) * PARAMS::BATCH_SIZE, 0);
                auto relevant_output_target_matrix_view = rlt::view(device, output_target_matrix_view, rlt::matrix::ViewSpec<PARAMS::MEM_SIZE * PARAMS::BATCH_SIZE, 1>{}, (PARAMS::MEM_SIZE + PARAMS::MEM_DELAY) * PARAMS::BATCH_SIZE, 0);
                T loss = rlt::nn::loss_functions::categorical_cross_entropy::evaluate(device, relevant_output_logits_matrix_view, relevant_output_target_matrix_view);
                last_print = std::chrono::high_resolution_clock::now();
                std::cout << "Epoch: " << epoch_i << " Sample: " << sample_i << " Batch: " << sample_i/PARAMS::BATCH_SIZE << " (" << sample_i/PARAMS::BATCH_SIZE/elapsed << " batch/s)" << " Loss: " << loss << std::endl;
            }
            rlt::zero_gradient(device, model);
            {
#ifdef RL_TOOLS_ENABLE_TRACY
                ZoneScopedN("backward");
#endif
                rlt::backward(device, model, input, d_output, buffer);
            }
            rlt::step(device, optimizer, model);
        }
    }
    return 0;
}
