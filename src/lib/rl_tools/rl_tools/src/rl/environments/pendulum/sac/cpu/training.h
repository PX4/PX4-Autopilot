#ifdef RL_TOOLS_BACKEND_ENABLE_BLAS
#define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#endif

struct TestTag{};


#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/extrack/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

using TEST_USE_CASE = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Accumulator, float>;

using TYPE_POLICY = rlt::numeric_types::Policy<float, TEST_USE_CASE>;
static_assert(rlt::utils::typing::is_same_v<TYPE_POLICY::GET<rlt::numeric_types::categories::Accumulator>, float>);



using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T_DEFAULT = float;
using TI = typename DEVICE::index_t;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T_DEFAULT, TI, rlt::rl::environments::pendulum::DefaultParameters<T_DEFAULT>>;
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
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP>;
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_CORE_CONFIG>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_EXTRACK_CONFIG>{
    static constexpr TI EVALUATION_EPISODES = 100;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_EXTRACK_CONFIG, LOOP_EVAL_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#endif

using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

void run(TI seed = 0){
    DEVICE device;
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, 0);
#if defined(RL_TOOLS_ENABLE_TENSORBOARD) && !defined(RL_TOOLS_DISABLE_TENSORBOARD)
        rlt::init(device, device.logger, ts.extrack_paths.seed);
#endif
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
    rlt::free(device, device.logger);
    rlt::free(device, ts);
}


// benchmark training should take < 2s on P1, < 0.75 on M3