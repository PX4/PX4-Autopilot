#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/memory/operations_cpu.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#ifdef RL_TOOLS_ENABLE_HDF5


#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>
#endif

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

#include "approximators.h"

using DEVICE = rlt::devices::DefaultCPU;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TI = typename DEVICE::index_t;

#include "parameters.h"

int main(){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    std::string checkpoint = "experiments/2024-08-28_11-19-30/e64577b_sequential_algorithm_environment/sac_memory/0001/steps/000000000010000/checkpoint.h5";
    using CONFIG = ConfigApproximatorsSequential<T, TI, SEQUENCE_LENGTH, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
    using CAPABILITY = rlt::nn::capability::Forward<>;
    using ACTOR = CONFIG::Actor<CAPABILITY>::MODEL;
    ACTOR actor;
    ACTOR::Buffer<1> actor_buffer;
    rlt::malloc(device, actor);
    rlt::malloc(device, actor_buffer);

    auto actor_file = HighFive::File(checkpoint, HighFive::File::ReadOnly);
    rlt::load(device, actor, actor_file.getGroup("actor"));

    for(TI repeat_i = 0; repeat_i < 2; repeat_i++){
        rlt::Mode<rlt::nn::layers::gru::StepByStepMode<TI, rlt::mode::Evaluation>> mode;
        mode.reset = true;
        mode.step = 0;
        rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, 1, 1>>> input, output;
        rlt::malloc(device, input);
        rlt::malloc(device, output);


        std::vector<T> input_values = {1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, };
        for(T input_value : input_values){
            rlt::set(device, input, input_value, 0, 0, 0);
            rlt::evaluate(device, actor, input, output, actor_buffer, rng, mode);
            std::cout << mode.step << ": " << input_value << " => " << rlt::get(device, output, 0, 0, 0) << std::endl;
            mode.reset = false;
            mode.step++;
        }
    }

    return 0;
}