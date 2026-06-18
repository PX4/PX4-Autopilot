
#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/rl/environments/car/operations_generic.h>
#include <rl_tools/rl/environments/car/operations_json.h>
#include <rl_tools/ui_server/client/operations_cpu.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using ENV_SPEC = rlt::rl::environments::car::SpecificationTrack<T, TI, 100, 100, 20>;
using ENVIRONMENT = rlt::rl::environments::CarTrack<ENV_SPEC>;
using ENVIRONMENT_EVALUATION = ENVIRONMENT;
using UI = rlt::ui_server::client::UIBuffered<ENVIRONMENT>;

struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI BATCH_SIZE = 512;
    struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
        static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.001;
        static constexpr TI N_EPOCHS = 2;
        static constexpr T GAMMA = 0.995;
        static constexpr T LAMBDA = 0.975;
        static constexpr bool ADAPTIVE_LEARNING_RATE = false;
    };
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 512;
    static constexpr TI N_ENVIRONMENTS = 32;
    static constexpr TI TOTAL_STEP_LIMIT = 10000000;
    static constexpr TI STEP_LIMIT = TOTAL_STEP_LIMIT/(ON_POLICY_RUNNER_STEPS_PER_ENV * N_ENVIRONMENTS) + 1;
    static constexpr TI EPISODE_STEP_LIMIT = 1000;
};
using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequential>;
template <typename NEXT>
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, NEXT>{
    static constexpr TI EVALUATION_INTERVAL = 5;
    static constexpr TI NUM_EVALUATION_EPISODES = 1;
    static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
    static constexpr TI EPISODE_STEP_LIMIT = 1000;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, LOOP_EVAL_PARAMETERS<LOOP_CORE_CONFIG>, UI>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;
//static constexpr T EXPLORATION_NOISE_MULTIPLE = 0.5;
//struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<T, TI>{
//};
//struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::Parameters<T, TI, ENVIRONMENT>{
//    using PPO_PARAMETERS = PPO_PARAMETERS;
//};
//using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<T, TI, RNG, ENVIRONMENT>;
//struct EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<T, TI, LOOP_CORE_CONFIG>{
//    static constexpr TI EVALUATION_INTERVAL = 1;
//    static constexpr TI NUM_EVALUATION_EPISODES = 1;
//    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
//    static constexpr TI EPISODE_STEP_LIMIT = 1000;
//};
//using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, EVAL_PARAMETERS, UI>;
//using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
//using LOOP_CONFIG = LOOP_TIMING_CONFIG;
//using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

struct State{
    DEVICE device;
    LOOP_STATE ts;
    bool mode_interactive = false;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM, false>> interactive_action;
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
    rlt::get_ref(state->device, state->ts.actor_optimizer.parameters, 0).alpha = 1e-3;
    rlt::get_ref(state->device, state->ts.critic_optimizer.parameters, 0).alpha = 1e-3;
    return state;
}

float step(State* state){
    T sleep = state->ts.env_eval_parameters.dt / 10;
    if(state->mode_interactive){
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<T> diff = now-state->previous_interactive_step;
        if(diff.count() > state->ts.env_eval_parameters.dt){
            rlt::set_state(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->ts.ui, state->interactive_state, state->interactive_action);
            rlt::step(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->interactive_state, state->interactive_action, state->interactive_next_state, state->ts.rng_eval);
            bool terminated = rlt::terminated(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->interactive_next_state, state->ts.rng_eval);
            if(terminated){
                rlt::sample_initial_parameters(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->ts.rng_eval);
                rlt::sample_initial_state(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->interactive_state, state->ts.rng_eval);
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
                for (TI env_i = 0; env_i < LOOP_CORE_PARAMETERS::N_ENVIRONMENTS; env_i++) {
                    auto& env = get(state->ts.on_policy_runner.environments, 0, env_i);
                    env.track[row_i][col_i] = message["data"][row_i][col_i];
                }
                state->ts.env_eval.track[row_i][col_i] = message["data"][row_i][col_i];
            }
        }
        rlt::init(state->device, state->ts.on_policy_runner, state->ts.envs, state->ts.env_parameters, state->ts.rng);
    } else {
        if (message["channel"] == "setAction") {
            if(!state->mode_interactive){
                rlt::sample_initial_parameters(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->ts.rng_eval);
                rlt::sample_initial_state(state->device, state->ts.env_eval, state->ts.env_eval_parameters, state->interactive_state, state->ts.rng_eval);
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
    rlt::free(state->device, state->ts);
    delete state;
}
