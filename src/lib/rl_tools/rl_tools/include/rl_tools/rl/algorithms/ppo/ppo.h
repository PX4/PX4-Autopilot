#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_PPO_PPO_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_PPO_PPO_H

#include "../../../rl/components/running_normalizer/running_normalizer.h"
#include "../../../utils/generic/typing.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms{
    namespace ppo{
        template<typename TYPE_POLICY, typename TI, TI T_BATCH_SIZE>
        struct DefaultParameters {
            using T = typename TYPE_POLICY::DEFAULT;
            static constexpr T GAMMA = 0.99;
            static constexpr T LAMBDA = 0.95;
            static constexpr T EPSILON_CLIP = 0.2;
            static constexpr T INITIAL_ACTION_STD = 0.5; // note this is NOT log(std) but actual std (log is applied at init)
            static constexpr bool LEARN_ACTION_STD = true;
            static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;
            static constexpr T ADVANTAGE_EPSILON = 1e-8;
            static constexpr bool NORMALIZE_ADVANTAGE = true;
            static constexpr bool ADAPTIVE_LEARNING_RATE = false;
            static constexpr T ADAPTIVE_LEARNING_RATE_POLICY_KL_THRESHOLD = 0.008;
            static constexpr T POLICY_KL_EPSILON = 1e-5;
            static constexpr T ADAPTIVE_LEARNING_RATE_DECAY = (T)1/(T)1.5;
            static constexpr T ADAPTIVE_LEARNING_RATE_MIN = 1e-6;
            static constexpr T ADAPTIVE_LEARNING_RATE_MAX = 1e-2;
            static constexpr TI N_WARMUP_STEPS_CRITIC = 0;
            static constexpr TI N_WARMUP_STEPS_ACTOR = 0;
            static constexpr TI N_EPOCHS = 10;
            static constexpr TI BATCH_SIZE = T_BATCH_SIZE;
            static constexpr bool IGNORE_TERMINATION = false; // ignoring the termination flag is useful for training on environments with negative rewards, where the agent would try to terminate the episode as soon as possible otherwise
        };

        template<
                typename T_TYPE_POLICY,
                typename T_TI,
                typename T_ENVIRONMENT,
                typename T_ACTOR_TYPE,
                typename T_CRITIC_TYPE,
                typename T_PARAMETERS
        >
        struct Specification {
            using TYPE_POLICY = T_TYPE_POLICY;
            using TI = T_TI;
            using ENVIRONMENT = T_ENVIRONMENT;
            using ACTOR_TYPE = T_ACTOR_TYPE;
            using CRITIC_TYPE = T_CRITIC_TYPE;
            using PARAMETERS = T_PARAMETERS;
            static constexpr bool ASYMMETRIC_OBSERVATIONS = !rl_tools::utils::typing::is_same_v<typename ENVIRONMENT::Observation, typename ENVIRONMENT::ObservationPrivileged>;

            static_assert(get_last(typename ACTOR_TYPE::INPUT_SHAPE{}) == ENVIRONMENT::Observation::DIM);
            static_assert(get_last(typename CRITIC_TYPE::INPUT_SHAPE{}) == ENVIRONMENT::ObservationPrivileged::DIM);
            static_assert(get_last(typename ACTOR_TYPE::OUTPUT_SHAPE{}) == ENVIRONMENT::ACTION_DIM);
            static_assert(get_last(typename CRITIC_TYPE::OUTPUT_SHAPE{}) == 1);
        };

        template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
        struct BufferSpecification{
            using SPEC = T_SPEC;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };
        template <typename T_BUFFER_SPEC>
        struct Buffers{
            using BUFFER_SPEC = T_BUFFER_SPEC;
            using SPEC = typename BUFFER_SPEC::SPEC;
            using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
            using TI = typename SPEC::TI;
            static constexpr TI BATCH_SIZE = SPEC::PARAMETERS::BATCH_SIZE;
            static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
            static constexpr TI OBSERVATION_DIM = SPEC::ENVIRONMENT::Observation::DIM;
            Matrix<matrix::Specification<T, TI, BATCH_SIZE, ACTION_DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>> current_batch_actions;
            Matrix<matrix::Specification<T, TI, BATCH_SIZE, 1, BUFFER_SPEC::DYNAMIC_ALLOCATION>> d_critic_output;
            Matrix<matrix::Specification<T, TI, BATCH_SIZE, ACTION_DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>> d_action_log_prob_d_action;
            Matrix<matrix::Specification<T, TI, BATCH_SIZE, ACTION_DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>> d_action_log_prob_d_action_log_std;
            Matrix<matrix::Specification<T, TI, 1, ACTION_DIM/SPEC::ENVIRONMENT::N_AGENTS, BUFFER_SPEC::DYNAMIC_ALLOCATION>> rollout_log_std;
        };
    }

    template<typename T_SPEC>
    struct PPO {
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;

        typename SPEC::ACTOR_TYPE actor;
        typename SPEC::CRITIC_TYPE critic;
#ifdef RL_TOOLS_DEBUG_RL_ALGORITHMS_PPO_CHECK_INIT
        bool initialized = false;
#endif

    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
