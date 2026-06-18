#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/extrack/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/checkpoint/config.h>
#include <rl_tools/rl/loop/steps/save_trajectories/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>


#include "../../../experiments/2024-12-30_11-02-28/9c7e166_zoo_environment_algorithm/l2f_sac/0000/steps/000000000000000/checkpoint.h"

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TI = typename DEVICE::index_t;

int main(){
    DEVICE device;
    RNG rng;
    rlt::init(device, rng, 0);

    using ACTOR_TYPE = rl_tools::checkpoint::actor::TYPE::CHANGE_BATCH_SIZE<TI, 1>;
    ACTOR_TYPE::Buffer<false> buffer;
    // test
    rl_tools::checkpoint::actor::TYPE::Buffer<> test_buffer;
    rlt::Tensor<rlt::tensor::Specification<T, TI, rl_tools::checkpoint::example::output::SHAPE>> output;
    rlt::malloc(device, test_buffer);
    rlt::malloc(device, output);
    rlt::Mode<rlt::mode::Evaluation<>> mode;
    rlt::evaluate(device, rl_tools::checkpoint::actor::module, rl_tools::checkpoint::example::input::container, output, test_buffer, rng, mode);
    T abs_diff = rlt::abs_diff(device, rl_tools::checkpoint::example::output::container, output) / rl_tools::checkpoint::example::output::SPEC::SIZE;
    auto last_step_output = rlt::view(device, output, rlt::get<0>(rl_tools::checkpoint::example::output::SHAPE{}) - 1);
    auto last_step_expected = rlt::view(device, rl_tools::checkpoint::example::output::container, rlt::get<0>(rl_tools::checkpoint::example::output::SHAPE{}) - 1);
    std::cout << "last_step_output: " << std::endl;
    rlt::print(device, last_step_output);
    std::cout << "last_step_expected: " << std::endl;
    rlt::print(device, last_step_expected);

    std::cout << "abs_diff to checkpoint example: " << abs_diff << std::endl;
    rlt::free(device, test_buffer);
    rlt::free(device, output);
    return abs_diff < 1e-5 ? 0 : 1;
}
