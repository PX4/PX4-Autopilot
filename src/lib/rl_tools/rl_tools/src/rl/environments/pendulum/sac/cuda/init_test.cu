#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/random/operations_cuda.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/dense/operations_cuda.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using DEVICE_CUDA = rlt::devices::DEVICE_FACTORY_CUDA<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr TI ACTOR_BATCH_SIZE = 100;
        static constexpr TI CRITIC_BATCH_SIZE = 100;
    };
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
};
#ifdef BENCHMARK
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CORE_CONFIG>{
    static constexpr TI EVALUATION_EPISODES = 100;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, LOOP_EVAL_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#endif

using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;


int main(){
    TI seed = 0;
    DEVICE device;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    LOOP_STATE ts;
    decltype(ts.actor_critic.actor) actor_cuda;
    DEVICE_CUDA::SPEC::RANDOM::ENGINE<> rng_cuda;
    rlt::malloc(device_cuda, rng_cuda);
    rlt::init(device_cuda, rng_cuda, 0);
    rlt::malloc(device_cuda, actor_cuda);
    rlt::malloc(device, ts);
    rlt::init(device, ts, 0);
    rlt::init_weights(device_cuda, actor_cuda, rng_cuda);
    rlt::copy(device_cuda, device, actor_cuda, ts.actor_critic.actor);
    while(!rlt::step(device, ts)){
#ifndef BENCHMARK
        if(ts.step == 5000){
            std::cout << "steppin yourself > callbacks 'n' hooks: " << ts.step << std::endl;
        }
#endif
#ifdef BENCHMARK_ABLATION_SIMULATOR
        std::this_thread::sleep_for(std::chrono::duration<T>(8.072980403900147e-05)); // python gymnasium Pendulum-v1 step time
#endif
    }
    rlt::free(device, ts);
}


// benchmark training should take < 2s on P1, < 0.75 on M3
