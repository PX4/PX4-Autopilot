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

#include <rl_tools/utils/extrack/operations_cpu.h>

namespace rlt = rl_tools;

#include "gru_model.h"


#include <chrono>
#include <filesystem>
#include <thread>

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
using CONFIG = Config<TYPE_POLICY, TI>;



int main() {
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device, rng, 0);


    rlt::utils::extrack::Path run;
    run.name = "gru-enwik";
    run.step = std::to_string(600000);
    run.require_checkpoint = true;
    bool found_run = rlt::find_latest_run(device, "experiments", run);
    if(found_run){
        std::cout << "found run: " << run.checkpoint_path << std::endl;
    }
    else{
        std::cout << "could not find run: " << run.checkpoint_path << std::endl;
        std::exit(1);
    }





    typename CONFIG::MODEL model;
    typename CONFIG::MODEL::Buffer<> buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<T, TI, typename CONFIG::MODEL::OUTPUT_SHAPE>> output;

    rlt::malloc(device, model);
    rlt::malloc(device, buffer);
    rlt::malloc(device, input);
    std::filesystem::path FILE_PATH = run.checkpoint_path;

    {
        auto file = HighFive::File(FILE_PATH.string(), HighFive::File::ReadOnly);
        auto checkpoint_group = rlt::get_group(device, file, "checkpoint");
        rlt::load(device, model, checkpoint_group);
    }

    std::string input_string;
//    std::getline(std::cin, input_string);
//    std::cout << "Input: " << input_string << std::endl;
    input_string = "The car is on the str";
    std::cout << input_string << std::flush;
    while(true){
        if(input_string.size() > CONFIG::PARAMS::SEQUENCE_LENGTH){
            input_string = input_string.substr(input_string.size() - CONFIG::PARAMS::SEQUENCE_LENGTH, CONFIG::PARAMS::SEQUENCE_LENGTH);
        }
        rlt::set_all(device, input, 0);
        for(TI batch_i = 0; batch_i < CONFIG::PARAMS::BATCH_SIZE; batch_i++){
            for(TI sequence_i = 0; sequence_i < CONFIG::PARAMS::SEQUENCE_LENGTH; sequence_i++){
                if(sequence_i < input_string.size()) {
                    char input_char = input_string[sequence_i];
                    if(input_char < 0) {
                        input_char = '?';
                    }
                    rlt::set(device, input, input_char, sequence_i, batch_i, 0);
                }
            }
        }

        rlt::forward(device, model, input, buffer, rng);
        auto output_matrix = rlt::output(device, model);
        output._data = output_matrix._data;
        auto sequence_step = rlt::view(device, output, input_string.size()-1);
        auto logits = rlt::view(device, sequence_step, 0);
        T temperature = 0.5;
        rlt::scale(device, logits, 1/temperature);
        rlt::exp(device, logits);
        T sum = rlt::sum(device, logits);
        T comulative_prob = 0;
        T random_number = rlt::random::uniform_real_distribution(device.random, (T)0, (T)1, rng);
        for(TI i=0; i < CONFIG::PARAMS::NUM_CLASSES; i++){
            T prob = rlt::get(device, logits, i) / sum;
            if(random_number < comulative_prob + prob){
                input_string += (char)i;
                std::cout << (char)i << std::flush;
                break;
            }
            comulative_prob += prob;
            if(i == CONFIG::PARAMS::NUM_CLASSES - 1){
                input_string += (char)i;
                std::cout << (char)0 << std::flush;
            }
        }
//        rlt::print(device, logits);
//        auto logit_matrix = rlt::matrix_view(device, logits);
//        char next_token = rlt::argmax_row(device, logit_matrix);
//        input_string += next_token;
//        std::cout << input_string << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}

