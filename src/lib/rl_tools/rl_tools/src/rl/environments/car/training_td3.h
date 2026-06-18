#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>

#ifdef LOAD_TRACK_FROM_FILE
#include <rl_tools/rl/environments/car/operations_cpu.h>
#else
#include <rl_tools/rl/environments/car/operations_generic.h>
#endif
#include <rl_tools/rl/environments/car/operations_json.h>
#include <rl_tools/ui_server/client/operations_cpu.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>


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
using UI = rlt::ui_server::client::UIBuffered<ENVIRONMENT>;

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
    static constexpr TI STEP_LIMIT = 10000000;
    static constexpr TI REPLAY_BUFFER_CAP = 20000;
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
    static constexpr TI EVALUATION_INTERVAL = 10000;
    static constexpr TI NUM_EVALUATION_EPISODES = 1;
    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
    static constexpr TI EPISODE_STEP_LIMIT = 1000;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, EVAL_PARAMETERS, UI>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

struct State{
    DEVICE device;
    LOOP_STATE ts;
    bool mode_interactive = false;
    rlt::MatrixStatic<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> interactive_action;
    bool mode_training = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous_interactive_step;
    ENVIRONMENT::State interactive_state, interactive_next_state;
    bool finished = false;
};

State* create(int seed){
    auto* state = new State();
    rlt::malloc(state->device, state->ts);
    rlt::init(state->device, state->ts, seed);
    rlt::set_all(state->device, state->interactive_action, 0);
    return state;
}

float step(State* state){
    T sleep = state->ts.env_eval.parameters.dt / 10;
    if(state->mode_interactive){
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<T> diff = now-state->previous_interactive_step;
        if(diff.count() > state->ts.env_eval.parameters.dt){
            rlt::set_state(state->device, state->ts.env_eval, state->ts.ui, state->interactive_state, state->interactive_action);
            rlt::step(state->device, state->ts.env_eval, state->interactive_state, state->interactive_action, state->interactive_next_state, state->ts.rng_eval);
            bool terminated = rlt::terminated(state->device, state->ts.env_eval, state->interactive_next_state, state->ts.rng_eval);
            if(terminated){
                rlt::sample_initial_state(state->device, state->ts.env_eval, state->interactive_state, state->ts.rng_eval);
            }
            else{
                state->interactive_state = state->interactive_next_state;
            }
            state->previous_interactive_step = now;
        }
    }
    else{
        if(state->mode_training){
            sleep = 0;
            state->finished = rlt::step(state->device, state->ts);
        }
    }
    return sleep;
}

float step(State* state, const char* message_string) {
    auto message = nlohmann::json::parse(message_string);
    if (message["channel"] == "setTrack") {
        for (TI row_i = 0; row_i < ENVIRONMENT::SPEC::HEIGHT; row_i++) {
            for (TI col_i = 0; col_i < ENVIRONMENT::SPEC::WIDTH; col_i++) {
                for (auto &env: state->ts.envs) {
                    env.parameters.track[row_i][col_i] = message["data"][row_i][col_i];
                }
                state->ts.env_eval.parameters.track[row_i][col_i] = message["data"][row_i][col_i];
            }
        }
        rlt::init(state->device, state->ts.off_policy_runner, state->ts.envs);
    } else {
        if (message["channel"] == "setAction") {
            if(!state->mode_interactive){
                rlt::sample_initial_state(state->device, state->ts.env_eval, state->interactive_state, state->ts.rng_eval);
            }
            rlt::set(state->interactive_action, 0, 0, message["data"][0]);
            rlt::set(state->interactive_action, 0, 1, message["data"][1]);
            state->previous_interactive_step = std::chrono::high_resolution_clock::now();
            state->mode_interactive = true;
        } else {
            if (message["channel"] == "startTraining") {
                state->mode_interactive = false;
                state->mode_training = true;
            }
        }
    }
    return step(state);
}
void destroy(State* state){
    delete state;
}
