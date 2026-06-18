#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/embedding/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/loss_functions/categorical_cross_entropy/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/embedding/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

namespace rlt = rl_tools;

#include "gru_model.h"


#include <chrono>
#include <filesystem>
#include <thread>
#include <fstream>

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using TI = typename DEVICE::index_t;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using CONFIG = Config<TYPE_POLICY, TI>;



int main() {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device, rng, 0);


    typename CONFIG::MODEL original_model;
    constexpr TI VALIDATION_BATCH_SIZE = 512;
    using CAPABILITY_EVALUATION = rlt::nn::capability::Forward<>;
    using FORWARD_MODEL = typename CONFIG::MODEL::template CHANGE_CAPABILITY<CAPABILITY_EVALUATION>;
    using MODEL = typename FORWARD_MODEL::template CHANGE_BATCH_SIZE<TI, VALIDATION_BATCH_SIZE>;
    MODEL model;
    MODEL::Buffer<> buffer;
    using TARGET_SHAPE = rlt::tensor::Replace<MODEL::OUTPUT_SHAPE, 1, 2>;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename MODEL::OUTPUT_SHAPE>> output;
    rlt::Tensor<rlt::tensor::Specification<T, TI, TARGET_SHAPE>> target;

    rlt::malloc(device, original_model);
    rlt::malloc(device, model);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, target);
    std::filesystem::path FILE_PATH = "model_checkpoint.h5";

    {
        auto file = HighFive::File(FILE_PATH.string(), HighFive::File::ReadOnly);
        auto checkpoint_group = rlt::get_group(device, file, "checkpoint");
        rlt::load(device, original_model, checkpoint_group);
    }
    rlt::copy(device, device, original_model, model);

    std::string dataset_file_name = "data/enwik8/enwik8-valid.txt";
    std::string data_path = dataset_file_name;
    std::string dataset_string;
    std::ifstream file(data_path);
    if(!file.is_open()){
        std::cerr << "Failed to open file: " << data_path << std::endl;
        std::exit(-1);
    }
    std::string line;
    while(std::getline(file, line)){
        dataset_string += line;
    }
    T xe = 0;
    TI total = 0;
    T correct = 0;
    for(TI current_index = 0; (current_index + CONFIG::PARAMS::SEQUENCE_LENGTH + VALIDATION_BATCH_SIZE + 1) < dataset_string.size(); current_index += VALIDATION_BATCH_SIZE * 100){
        for(TI sequence_i = 0; sequence_i < CONFIG::PARAMS::SEQUENCE_LENGTH; sequence_i++){
            for(TI batch_i = 0; batch_i < VALIDATION_BATCH_SIZE; batch_i++){
                char input_char = dataset_string[current_index + sequence_i + batch_i];
                char output_char = dataset_string[current_index + sequence_i + batch_i + 1];
                if(input_char < 0) {
                    input_char = '?';
                }
                if(output_char < 0) {
                    output_char = '?';
                }
                rlt::set(device, input, input_char, sequence_i, batch_i, 0);
                rlt::set(device, target, output_char, sequence_i, batch_i, 0);
            }
        }
        rlt::evaluate(device, model, input, output, buffer, rng);
        auto output_matrix_view = rlt::matrix_view(device, output);
        auto target_matrix_view = rlt::matrix_view(device, target);
        T xe_step = rlt::nn::loss_functions::categorical_cross_entropy::evaluate(device, output_matrix_view, target_matrix_view);
        std::cout << "Loss: " << xe_step << std::endl;
        for(TI batch_i = 0; batch_i < VALIDATION_BATCH_SIZE; batch_i++){
            char output_char = dataset_string[current_index + CONFIG::PARAMS::SEQUENCE_LENGTH + batch_i];
            if(output_char < 0) {
                output_char = '?';
            }
            T max_logit = 0;
            bool max_logit_set = false;
            for(TI char_i = 0; char_i < CONFIG::PARAMS::NUM_CLASSES; char_i++){
                T logit = rlt::get(device, output, CONFIG::PARAMS::SEQUENCE_LENGTH - 1, batch_i, char_i);
                if(!max_logit_set || logit > max_logit){
                    max_logit = logit;
                    max_logit_set = true;
                }
            }
            T denominator = 0;
            T argmax_logit = 0;
            T argmax_i = 0;
            bool argmax_set = false;
            for(TI char_i = 0; char_i < CONFIG::PARAMS::NUM_CLASSES; char_i++){
                T logit = rlt::get(device, output, CONFIG::PARAMS::SEQUENCE_LENGTH - 1, batch_i, char_i);
                denominator += rlt::math::exp(device.math, logit - max_logit);
                if(!argmax_set || logit > argmax_logit){
                    argmax_logit = logit;
                    argmax_i = char_i;
                    argmax_set = true;
                }
            }
            if(argmax_i == output_char){
                correct++;
            }

            denominator = rlt::math::log(device.math, denominator) + max_logit;
            T numerator = rlt::get(device, output, CONFIG::PARAMS::SEQUENCE_LENGTH - 1, batch_i, output_char);
            xe += - numerator + denominator;
            total++;
        }
        T bpc = xe / total / rlt::math::log(device.math, (T)2);
//        std::cout << "Cross entropy per character: " << xe / xe_points << std::endl;
        std::cout << "Bits per byte: " << bpc << " Accuracy: " << correct / total << std::endl;
    }
    return 0;
}
