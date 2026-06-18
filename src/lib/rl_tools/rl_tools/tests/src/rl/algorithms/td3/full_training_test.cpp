#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/td3/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/td3/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
// using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using TI = typename DEVICE::index_t;


using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<TYPE_POLICY::DEFAULT, TI, rlt::rl::environments::pendulum::DefaultParameters<TYPE_POLICY::DEFAULT>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
};

using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

#include <gtest/gtest.h>
#ifdef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_DEBUG
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_FULL_TRAINING, TEST_FULL_TRAINING_DEBUG) {
#else
TEST(RL_TOOLS_RL_ALGORITHMS_TD3_FULL_TRAINING, TEST_FULL_TRAINING) {
#endif
    DEVICE device;
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts);
    auto start_time = std::chrono::high_resolution_clock::now();
    while(!rlt::step(device, ts)){
        if(ts.step == 5000){
            std::cout << "steppin yourself > callbacks 'n' hooks: " << ts.step << std::endl;
        }
#ifdef RL_TOOLS_TESTS_CODE_COVERAGE
        if(ts.step > LOOP_CORE_PARAMETERS::N_WARMUP_STEPS*2){
            break;
        }
#endif
    }
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - start_time;
    std::cout << "total time: " << elapsed_seconds.count() << "s" << std::endl;
    if(std::getenv("RL_TOOLS_TEST_ENABLE_TIMING") != nullptr){
#ifndef RL_TOOLS_TEST_RL_ALGORITHMS_TD3_FULL_TRAINING_DEBUG
#ifdef RL_TOOLS_TEST_MACHINE_LENOVO_P1
        ASSERT_LT(elapsed_seconds.count(), 6); // should be 5.5s when run in isolation
#endif
#ifdef RL_TOOLS_TEST_MACHINE_MACBOOK_M1
        ASSERT_LT(elapsed_seconds.count(), 3); // should be 2.5s when run in isolation
#endif
#endif
    }
    rlt::free(device, ts);
}
