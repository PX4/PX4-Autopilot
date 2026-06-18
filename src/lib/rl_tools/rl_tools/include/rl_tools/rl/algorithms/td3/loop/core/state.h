#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_LOOP_CORE_STATE_H

#include "../../../../../rl/algorithms/td3/td3.h"
#include "../../../../../rl/components/off_policy_runner/off_policy_runner.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::algorithms::td3::loop::core{
        // Config State (Init/Step)
        template<typename T_CONFIG>
        struct State{
            using CONFIG = T_CONFIG;
            using TI = typename CONFIG::TI;
            typename CONFIG::RNG rng;
            rl::components::OffPolicyRunner<typename CONFIG::OFF_POLICY_RUNNER_SPEC> off_policy_runner;
            typename CONFIG::ENVIRONMENT envs[decltype(off_policy_runner)::N_ENVIRONMENTS];
            typename CONFIG::ENVIRONMENT::Parameters env_parameters[decltype(off_policy_runner)::N_ENVIRONMENTS];
            typename CONFIG::ACTOR_CRITIC_TYPE actor_critic;
            // typename CONFIG::NN::ACTOR_TYPE::template Buffer<CONFIG::DYNAMIC_ALLOCATION> actor_deterministic_evaluation_buffers;
//            rl::components::off_policy_runner::Batch<rl::components::off_policy_runner::BatchSpecification<typename decltype(off_policy_runner)::SPEC, CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>> critic_batch;
            rl::components::off_policy_runner::SequentialBatch<typename CONFIG::CRITIC_BATCH_SPEC> critic_batch;
            rl::algorithms::td3::CriticTrainingBuffers<rl::algorithms::td3::CriticTrainingBuffersSpecification<typename CONFIG::ACTOR_CRITIC_SPEC, CONFIG::DYNAMIC_ALLOCATION>> critic_training_buffers;
            typename CONFIG::NN::CRITIC_TYPE::template Buffer<CONFIG::DYNAMIC_ALLOCATION> critic_buffers[2];
            using TARGET_CRITIC = typename rl_tools::utils::typing::remove_reference_t<decltype(actor_critic.critics_target[0])>::template CHANGE_SEQUENCE_LENGTH<TI, CONFIG::TARGET_SEQUENCE_LENGTH>;
            typename TARGET_CRITIC::template Buffer<CONFIG::DYNAMIC_ALLOCATION> critic_target_buffers[2];
            rl::components::off_policy_runner::SequentialBatch<typename CONFIG::ACTOR_BATCH_SPEC> actor_batch;
//            rl::components::off_policy_runner::Batch<rl::components::off_policy_runner::BatchSpecification<typename decltype(off_policy_runner)::SPEC, CONFIG::ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>> actor_batch;
            rl::algorithms::td3::ActorTrainingBuffers<rl::algorithms::td3::ActorTrainingBuffersSpecification<typename CONFIG::ACTOR_CRITIC_TYPE::SPEC, CONFIG::DYNAMIC_ALLOCATION>> actor_training_buffers;
            typename CONFIG::NN::ACTOR_TYPE::template Buffer<CONFIG::DYNAMIC_ALLOCATION> actor_buffers[2];
            using TARGET_ACTOR = typename decltype(actor_critic.actor)::template CHANGE_SEQUENCE_LENGTH<TI, CONFIG::TARGET_SEQUENCE_LENGTH>;
            typename TARGET_ACTOR::template Buffer<CONFIG::DYNAMIC_ALLOCATION> actor_target_buffers[2];
            typename CONFIG::EVAL_ACTOR_TYPE::template Buffer<CONFIG::DYNAMIC_ALLOCATION> actor_buffers_eval;
            TI step;
        };
    }
    template <typename T_CONFIG>
    constexpr auto& get_actor(rl::algorithms::td3::loop::core::State<T_CONFIG>& ts){
        return ts.actor_critic.actor;
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif