// #define DEBUG_BARE
#ifdef DEBUG_BARE
#include <iostream>
#endif

#include <rl_tools/operations/arm.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/operations_generic.h>

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/td3/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
// #include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/td3/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
// #include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultARM;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
static constexpr bool DYNAMIC_ALLOCATION = false;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
};
using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_EVAL_CONFIG;
using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

int main(int argc, char** argv){
    DEVICE device;
    TI seed = 3;
    if(argc > 1){
        seed = 10; //std::atoi(argv[1]);
    }
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
    while(!rlt::step(device, ts)){}

    using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, ENVIRONMENT, LOOP_EVAL_CONFIG::EVALUATION_PARAMETERS::NUM_EVALUATION_EPISODES, LOOP_EVAL_CONFIG::CORE_PARAMETERS::EPISODE_STEP_LIMIT>;
    rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
    typename decltype(ts.actor_critic.actor)::template State<false> state;
    rlt::malloc(device, ts);
    rlt::evaluate(device, ts.env_eval, ts.ui, ts.actor_critic.actor, result, ts.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
#ifdef DEBUG_BARE
    std::cout << "Mean return: " << result.returns_mean << std::endl;
#endif
    TI return_code = (TI)(-result.returns_mean / 100);
    rlt::free(device, ts);
    return return_code < 4 ? 0 : return_code;
}
