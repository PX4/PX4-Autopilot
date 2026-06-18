#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>

#include <rl_tools/rl/environments/car/operations_cpu.h>
#if RL_TOOLS_ENABLE_GTK
#include <rl_tools/rl/environments/car/ui.h>
#else
#include <rl_tools/rl/environments/car/operations_json.h>
#include <rl_tools/ui_server/client/operations_boost.h>
#endif
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
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
using T = float;
using TI = typename DEVICE::index_t;

using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, TI, 100, 100, 20>;
using ENVIRONMENT = rlt::rl::environments::CarTrack<ENV_SPEC>;
using ENVIRONMENT_EVALUATION = ENVIRONMENT;
#if RL_TOOLS_ENABLE_GTK
        using UI = rlt::rl::environments::car::UI<rlt::rl::environments::car::ui::Specification<T, TI, ENVIRONMENT, 1000, 60>>;
#else
        using UI = rlt::ui_server::client::UI<ENVIRONMENT>;
#endif

static constexpr T EXPLORATION_NOISE_MULTIPLE = 0.5;
struct TD3_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<T, TI>{
    constexpr static TI CRITIC_BATCH_SIZE = 256;
    constexpr static TI ACTOR_BATCH_SIZE = 256;
    static constexpr int N_WARMUP_STEPS_ACTOR = 10000;
    static constexpr int N_WARMUP_STEPS_CRITIC = 10000;
    constexpr static T GAMMA = 0.99;
    static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.2 * EXPLORATION_NOISE_MULTIPLE;
    static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.5 * EXPLORATION_NOISE_MULTIPLE;
    static constexpr TI CRITIC_TRAINING_INTERVAL = 10;
    static constexpr TI ACTOR_TRAINING_INTERVAL = 20;
    static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 20;
    static constexpr TI ACTOR_TARGET_UPDATE_INTERVAL = 20;
};
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<T, TI, ENVIRONMENT>{
    using TD3_PARAMETERS = TD3_PARAMETERS;
    static constexpr TI STEP_LIMIT = 1000000;
    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI EPISODE_STEP_LIMIT = 500;
};
using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsMLP>;
struct EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<T, TI, LOOP_CORE_CONFIG>{
    static constexpr TI EVALUATION_INTERVAL = 20000;
    static constexpr TI NUM_EVALUATION_EPISODES = 1;
    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
    static constexpr TI EPISODE_STEP_LIMIT = 1000;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, EVAL_PARAMETERS, UI>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

int main(int argc, char** argv) {
    DEVICE device;
    TI seed = 0;
    if (argc > 1) {
        seed = std::atoi(argv[1]);
    }
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
    while(!rlt::step(device, ts)){
    }
    rlt::free(device, ts);
    return 0;
}
