

// #define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#ifdef RL_TOOLS_ENABLE_TRACY
#include "Tracy.hpp"
#endif

#include <rl_tools/operations/cpu_mux.h>


template <typename DEVICE, typename OBJECT>
void check(DEVICE& device, const OBJECT& object, std::string name){
    rl_tools::utils::assert_exit(device, !is_nan(device, object) && is_finite(device, object), name + " is nan");
}



#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/parameters/reward_functions/squared/operations_generic.h>
#include <rl_tools/rl/environments/l2f/parameters/reward_functions/default.h>
#include <rl_tools/rl/environments/l2f/parameters/default.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/crazyflie.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/arpl.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/x500_sim.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/x500_real.h>
#include <rl_tools/rl/environments/l2f/parameters/init/default.h>
#include <rl_tools/rl/environments/l2f/parameters/termination/default.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
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
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

// #include "approximators.h"


using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

#include "parameters.h"


using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

static_assert(ENVIRONMENT::EPISODE_STEP_LIMIT >= SEQUENCE_LENGTH, "Environment episode step limit should be greater than or equal to sequence length");

int main(){
    TI seed = 1;
    DEVICE device;
    LOOP_STATE ts;
    ts.extrack_config.name = "sequential";
    ts.extrack_config.population_variates = "algorithm_environment_seq-len";
    ts.extrack_config.population_values = std::string("sac_l2f_") + std::to_string(SEQUENCE_LENGTH);
    rlt::malloc(device);
    rlt::init(device);
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
    std::cout << "Sizeof training state: " << sizeof(ts) << std::endl;
    DEVICE::SPEC::RANDOM::ENGINE<> myrng;
    rlt::malloc(device, myrng);
    rlt::init(device, myrng, 0);
#ifdef RL_TOOLS_ENABLE_TENSORBOARD
    rlt::init(device, device.logger, ts.extrack_paths.seed);
#endif
    bool done = false;
    while(!done){
#ifdef RL_TOOLS_ENABLE_TRACY
        FrameMark;
#endif
        done = rlt::step(device, ts);
    }
    return 0;
}
