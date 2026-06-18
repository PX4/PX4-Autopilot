#include <rl_tools/operations/wasm32.h>

#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include "/Users/jonas/rl_tools/experiments/2024-07-09_14-07-45/5676254_zoo_algorithm_environment/sac_pendulum-v1/0000/steps/000000000020000/checkpoint.h"

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultWASM32;
using T = rlt::checkpoint::actor::MODEL::T;
using TI = DEVICE::index_t;
constexpr TI BATCH_SIZE = 1;
rlt::MatrixStatic<rlt::matrix::Specification<T, TI, BATCH_SIZE, rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM>> output;
rlt::MatrixStatic<rlt::matrix::Specification<T, TI, BATCH_SIZE, rl_tools::checkpoint::actor::MODEL::INPUT_DIM>> input;
rlt::checkpoint::actor::MODEL::Buffer<BATCH_SIZE, rlt::MatrixStaticTag> buffer;

extern "C" {
    TI batch_size();
    TI input_dim();
    TI output_dim();
    void set_input(int32_t row, int32_t col, T value);
    T get_output(int32_t row, int32_t col);
    void evaluate();

    // example
    int32_t example_batch_size();
    T get_example_input(int32_t row, int32_t col);
    T get_example_output(int32_t row, int32_t col);
    T test();
}
TI batch_size(){
    return BATCH_SIZE;
}
TI input_dim(){
    return rl_tools::checkpoint::actor::MODEL::INPUT_DIM;
}
TI output_dim(){
    return rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM;
}

int32_t example_batch_size(){
    return rlt::checkpoint::example::input::CONTAINER_TYPE::ROWS;
}
T get_example_input(int32_t row, int32_t col){
    return rlt::get(rlt::checkpoint::example::input::container, row, col);
}
T get_example_output(int32_t row, int32_t col){
    return rlt::get(rlt::checkpoint::example::output::container, row, col);
}

void set_input(int32_t row, int32_t col, T value){
    rlt::set(input, row, col, value);
}

T get_output(int32_t row, int32_t col){
    return rlt::get(output, row, col);
}

void evaluate(){
    DEVICE device;
    bool rng;
    rlt::evaluate(device, rlt::checkpoint::actor::module, input, output, buffer, rng);
}


T test(){
    DEVICE device;
    bool rng;
    rlt::evaluate(device, rlt::checkpoint::actor::module, rlt::checkpoint::example::input::container, output, buffer, rng);
    return rlt::abs_diff(device, rlt::checkpoint::example::output::container, output);
}

