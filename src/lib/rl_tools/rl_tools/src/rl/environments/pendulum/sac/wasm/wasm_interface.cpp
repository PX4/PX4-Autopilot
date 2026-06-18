#include "full_training.h"
#include <emscripten.h>

using TRAINING_STATE = LOOP_STATE;
DEVICE device;
extern "C" {
    EMSCRIPTEN_KEEPALIVE
    TRAINING_STATE* proxy_create_training_state(int seed){
        TRAINING_STATE* ts = new TRAINING_STATE{};
        rlt::malloc(device, *ts);
        rlt::init(device, *ts, seed+5);
        return ts;
    }

    EMSCRIPTEN_KEEPALIVE
    int proxy_training_step(TRAINING_STATE* ts){
        return rlt::step(device, *ts);
    }

    EMSCRIPTEN_KEEPALIVE
    int proxy_get_step(TRAINING_STATE* ts){
        return ts->step;
    }

EMSCRIPTEN_KEEPALIVE
int proxy_get_state_dim(){
    return ENVIRONMENT::State::DIM;
}

EMSCRIPTEN_KEEPALIVE
double proxy_get_state_value(TRAINING_STATE* ts, int env_index, int state_index){
    using T = typename TRAINING_STATE::CONFIG::TYPE_POLICY::DEFAULT;
    static_assert(TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS == 1);
    if(env_index < TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS && state_index < TRAINING_STATE::CONFIG::ENVIRONMENT::State::DIM){
        auto& rb = rlt::get(ts->off_policy_runner.replay_buffers, 0, env_index);
        if(rb.position > 0){
            auto& env = rlt::get(ts->off_policy_runner.envs, 0, env_index);
            T c = rlt::get(rb.observations, rb.position - 1, 0);
            T s = rlt::get(rb.observations, rb.position - 1, 1);
            T theta = rlt::math::atan2(device.math, s, c);
            return theta;
        }
        else{
            return 0;
        }
    }
    else{
        return -1337;
    }
}

EMSCRIPTEN_KEEPALIVE
int proxy_get_episode(TRAINING_STATE* ts, int env_index){
    static_assert(TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS == 1);
    if(env_index < TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS){
        auto& es = rlt::get(ts->off_policy_runner.episode_stats, 0, env_index);
        auto episode = es.next_episode_i;
        return episode;
    }
    else{
        return -1337;
    }
}

EMSCRIPTEN_KEEPALIVE
double proxy_get_episode_return(TRAINING_STATE* ts, int env_index, int episode_i){
    static_assert(TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS == 1);
    if(env_index < TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS && episode_i < TRAINING_STATE::CONFIG::OFF_POLICY_RUNNER_SPEC::PARAMETERS::EPISODE_STATS_BUFFER_SIZE){
        auto& es = rlt::get(ts->off_policy_runner.episode_stats, 0, env_index);
        return rlt::get(es.returns, episode_i, 0);
    }
    else{
        return -1337;
    }
}

EMSCRIPTEN_KEEPALIVE
    int proxy_get_evaluation_count(){
#ifdef RL_TOOLS_ENABLE_EVALUATION
        return TRAINING_STATE::N_EVALUATIONS;
#endif
        return 0;
    }

EMSCRIPTEN_KEEPALIVE
    double proxy_get_evaluation_return(TRAINING_STATE* ts, int index){
#ifdef RL_TOOLS_ENABLE_EVALUATION
        return ts->evaluation_returns[index];
#endif
        return 0;
    }

EMSCRIPTEN_KEEPALIVE
    void proxy_destroy_training_state(TRAINING_STATE* ts){
        rlt::free(device, *ts);
        delete ts;
    }
}