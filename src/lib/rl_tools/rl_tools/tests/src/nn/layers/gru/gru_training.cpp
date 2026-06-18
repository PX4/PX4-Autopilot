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

#include <rl_tools/utils/extrack/operations_cpu.h>

#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/embedding/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

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
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif
using TI = typename DEVICE::index_t;
using T = float;

using CONFIG = Config<TYPE_POLICY, TI>;

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
    TI seed = 0;
    rlt::init(device);
    rlt::init(device, rng, seed);
    rlt::utils::extrack::Config<TI> extrack_config;
    extrack_config.name = "gru-enwik";
    rlt::utils::extrack::Paths extrack_paths;

    rlt::init(device, extrack_config, extrack_paths, seed);
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
        rlt::init(device, device.logger, extrack_paths.seed);
#endif

    std::string data_path = "data/enwik8/enwik8-training.txt";
    if(!std::filesystem::exists(data_path)){
        std::cerr << "Data path does not exist: " << data_path << std::endl;
        return 1;
    }
    std::string dataset_string;
    std::ifstream data_file(data_path);
    if(!data_file.is_open()){
        std::cerr << "Failed to open file: " << data_path << std::endl;
        std::exit(-1);
    }
    std::string line;
    while(std::getline(data_file, line)){
        dataset_string += line;
    }

    std::vector<std::tuple<std::string, std::string>> dataset;
    for(TI offset=0; offset < dataset_string.size() - CONFIG::PARAMS::SEQUENCE_LENGTH - 1; offset += CONFIG::PARAMS::SEQUENCE_LENGTH){
        auto input = dataset_string.substr(offset, CONFIG::PARAMS::SEQUENCE_LENGTH);
        auto output = dataset_string.substr(offset+1, CONFIG::PARAMS::SEQUENCE_LENGTH);
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
    std::filesystem::path resume_checkpoint;
    // std::filesystem::path resume_checkpoint = "experiments/2025-02-04_15-11-42/2895169_gru-enwik_default/default/0000/steps/000000000600000/checkpoint.h5";
    TI epoch_i = 0;
    TI sample_i = 0;
    bool resuming_from_checkpoint = false;
    decltype(rng.state) shuffle_rng_state_checkpoint = 0;
    if (!resume_checkpoint.empty()){
        auto file = HighFive::File(resume_checkpoint.string(), HighFive::File::ReadOnly);
        auto checkpoint_group = rlt::get_group(device, file, "checkpoint");
        rlt::load(device, model, checkpoint_group);
        epoch_i = rlt::get_attribute_int<TI>(device, checkpoint_group, "epoch");
        sample_i = rlt::get_attribute_int<TI>(device, checkpoint_group, "sample_i");
        resuming_from_checkpoint = true;
        auto rng_state_string = rlt::get_attribute(device, checkpoint_group, "rng_state");
        rng.state = std::stoull(rng_state_string);
        auto shuffle_rng_state_string = rlt::get_attribute(device, checkpoint_group, "shuffle_rng_state");
        shuffle_rng_state_checkpoint = std::stoull(shuffle_rng_state_string);
        rlt::set(device, optimizer.age, rlt::get_attribute_int<TI>(device, checkpoint_group, "optimizer_state"), 0);
    }
    for(; epoch_i < 1000; epoch_i++){
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_print = start_time;
        decltype(rng) shuffle_rng = rng;
        if (!resuming_from_checkpoint){
            sample_i = 0;
        }
        else{
            resuming_from_checkpoint = false;
            shuffle_rng.state = shuffle_rng_state_checkpoint;
        }
        decltype(rng.state) shuffle_rng_state = shuffle_rng.state;
        std::vector<TI> dataset_indices(dataset.size());
        std::iota(dataset_indices.begin(), dataset_indices.end(), 0);
        shuffle(device, dataset_indices.begin(), dataset_indices.end(), shuffle_rng);
        for(; sample_i < dataset.size() - CONFIG::PARAMS::BATCH_SIZE; sample_i += CONFIG::PARAMS::BATCH_SIZE){
            TI global_sample_i = epoch_i * ((TI)((dataset.size() - CONFIG::PARAMS::BATCH_SIZE) / CONFIG::PARAMS::BATCH_SIZE)) * CONFIG::PARAMS::BATCH_SIZE + sample_i;
            rlt::set_step(device, device.logger, global_sample_i);
#ifdef RL_TOOLS_ENABLE_TRACY
            FrameMark;
#endif
            if(sample_i % 10000 == 0){
                //checkpoint
#ifdef RL_TOOLS_ENABLE_HDF5
                auto checkpoint_folder = rlt::get_step_folder(device, extrack_config, extrack_paths, global_sample_i);
                std::filesystem::path checkpoint_path = checkpoint_folder / "checkpoint.h5";
                {
                    std::cout << "Checkpointing to: " << checkpoint_path << std::endl;
                    auto file = HighFive::File(checkpoint_path.string(), HighFive::File::Overwrite);
                    rlt::zero_gradient(device, model);
                    rlt::reset_forward_state(device, model);
                    auto checkpoint_group = rlt::create_group(device, file, "checkpoint");
                    rlt::set_attribute(device, checkpoint_group, "epoch", std::to_string(epoch_i).c_str());
                    rlt::set_attribute(device, checkpoint_group, "sample_i", std::to_string(sample_i).c_str());
                    rlt::set_attribute(device, checkpoint_group, "rng_state", std::to_string(rng.state).c_str());
                    rlt::set_attribute(device, checkpoint_group, "shuffle_rng_state", std::to_string(shuffle_rng_state).c_str());
                    rlt::set_attribute(device, checkpoint_group, "optimizer_state", std::to_string(rlt::get(device, optimizer.age, 0)).c_str());
                    rlt::save(device, model, checkpoint_group);
                }
                if(sample_i == 0 || sample_i == CONFIG::PARAMS::BATCH_SIZE){ // reload check
                    auto file = HighFive::File(checkpoint_path.string(), HighFive::File::ReadOnly);
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

            for(TI batch_i = 0; batch_i < CONFIG::PARAMS::BATCH_SIZE; batch_i++){
                for(TI sequence_i = 0; sequence_i < CONFIG::PARAMS::SEQUENCE_LENGTH; sequence_i++){
                    TI actual_sample_i = dataset_indices[sample_i + batch_i];
                    char input_char = std::get<0>(dataset[actual_sample_i])[sequence_i];
                    char output_char = std::get<1>(dataset[actual_sample_i])[sequence_i];
                    if(input_char < 0) {
                        input_char = '?';
                    }
                    if(output_char < 0) {
                        output_char = '?';
                    }
                    rlt::set(device, input, input_char, sequence_i, batch_i, 0);
                    rlt::set(device, output_target, output_char, sequence_i, batch_i, 0);
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
            T elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() / 1000.0;
            T elapsed_print = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_print).count() / 1000.0;
            if(sample_i % 100 == 0){
            // if(elapsed_print > 0.2 || sample_i % 10000 == 0){
                T loss = rlt::nn::loss_functions::categorical_cross_entropy::evaluate(device, output_logits_matrix_view, output_target_matrix_view);
                last_print = std::chrono::high_resolution_clock::now();
                std::cout << "Global Sample: " << global_sample_i << " Epoch: " << epoch_i << " Sample: " << sample_i << " Batch: " << sample_i/CONFIG::PARAMS::BATCH_SIZE << " (" << sample_i/CONFIG::PARAMS::BATCH_SIZE/elapsed << " batch/s)" << " Loss: " << loss << " Bits-per-byte: " << loss/rlt::math::log(device.math, (T)2) << std::endl;
                rlt::add_scalar(device, device.logger, "loss", loss);
                rlt::add_scalar(device, device.logger, "bits_per_byte", loss/rlt::math::log(device.math, (T)2));
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
