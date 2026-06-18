#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_PPO_LOOP_CORE_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_PPO_LOOP_CORE_STATE_H

#include "../../../../../rl/algorithms/ppo/ppo.h"
#include "../../../../../rl/components/off_policy_runner/off_policy_runner.h"
#include "../../../../../rl/environments/environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::algorithms::ppo::loop::core{
        // Config State (Init/Step)
        template<typename T_CONFIG>
        struct State{
            using CONFIG = T_CONFIG;
            using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
            using TI = typename CONFIG::TI;
            typename CONFIG::NN::ACTOR_OPTIMIZER actor_optimizer;
            typename CONFIG::NN::CRITIC_OPTIMIZER critic_optimizer;
            typename CONFIG::RNG rng;
            typename CONFIG::PPO_TYPE ppo;
            typename CONFIG::PPO_BUFFERS_TYPE ppo_buffers;
            typename CONFIG::ON_POLICY_RUNNER_TYPE on_policy_runner;
            typename CONFIG::ON_POLICY_RUNNER_DATASET_TYPE on_policy_runner_dataset;
            typename CONFIG::ACTOR_EVAL_BUFFERS actor_eval_buffers;
            typename CONFIG::ACTOR_BUFFERS actor_buffers;
            typename CONFIG::CRITIC_BUFFERS critic_buffers;
            typename CONFIG::CRITIC_BUFFERS_GAE critic_buffers_gae;
            using T = typename TYPE_POLICY::DEFAULT;
            Matrix<matrix::Specification<T, TI, CONFIG::ON_POLICY_RUNNER_DATASET_TYPE::STEPS_TOTAL, CONFIG::ENVIRONMENT::Observation::DIM, CONFIG::DYNAMIC_ALLOCATION>> observations_dense;
            rl::components::RunningNormalizer<rl::components::running_normalizer::Specification<TYPE_POLICY, TI, CONFIG::ENVIRONMENT::Observation::DIM/CONFIG::ENVIRONMENT::N_AGENTS, CONFIG::DYNAMIC_ALLOCATION>> observation_normalizer;
            rl::components::RunningNormalizer<rl::components::running_normalizer::Specification<TYPE_POLICY, TI, CONFIG::ENVIRONMENT::ObservationPrivileged::DIM, CONFIG::DYNAMIC_ALLOCATION>> observation_privileged_normalizer;
            typename CONFIG::ENVIRONMENT envs[CONFIG::CORE_PARAMETERS::N_ENVIRONMENTS];
            typename CONFIG::ENVIRONMENT::Parameters env_parameters[CONFIG::CORE_PARAMETERS::N_ENVIRONMENTS];
//            MatrixDynamic<matrix::Specification<typename CONFIG::T, TI, 1, CONFIG::ENVIRONMENT::Observation::DIM>> observations_mean, observations_std;
            environments::DummyUI ui;
            TI next_checkpoint_id = 0;
            TI next_evaluation_id = 0;
            TI step;
        };
    }
    template <typename T_CONFIG>
    constexpr auto& get_actor(rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        return ts.ppo.actor;
    }
    template <typename T_CONFIG>
    constexpr auto& get_actor(const rl::algorithms::ppo::loop::core::State<T_CONFIG>& ts){
        return ts.ppo.actor;
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif