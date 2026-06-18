#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_TD3_TD3_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_TD3_TD3_H


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::td3 {
    template<typename TYPE_POLICY, typename TI>
    struct DefaultParameters {
        using T = typename TYPE_POLICY::DEFAULT;
        static constexpr T GAMMA = 0.99;
        static constexpr TI ACTOR_BATCH_SIZE = 100;
        static constexpr TI CRITIC_BATCH_SIZE = 100;
        static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
        static constexpr TI ACTOR_TRAINING_INTERVAL = 2;
        static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 2;
        static constexpr TI ACTOR_TARGET_UPDATE_INTERVAL = 2;
        static constexpr T ACTOR_POLYAK = 1.0 - 0.005;
        static constexpr T CRITIC_POLYAK = 1.0 - 0.005;
        static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.2;
        static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.5;
        static constexpr bool IGNORE_TERMINATION = false; // ignoring the termination flag is useful for training on environments with negative rewards, where the agent would try to terminate the episode as soon as possible otherwise
        static constexpr TI SEQUENCE_LENGTH = 1;
        static constexpr bool MASK_NON_TERMINAL = true;
    };

    template<
        typename T_TYPE_POLICY,
        typename T_TI,
        typename T_ENVIRONMENT,
        typename T_ACTOR_TYPE,
        typename T_ACTOR_TARGET_TYPE,
        typename T_CRITIC_TYPE,
        typename T_CRITIC_TARGET_TYPE,
        typename T_OPTIMIZER,
        typename T_PARAMETERS,
        bool T_INCLUDE_FIRST_STEP_IN_TARGETS = false
    >
    struct Specification {
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        using ENVIRONMENT = T_ENVIRONMENT;
        using ACTOR_TYPE = T_ACTOR_TYPE;
        using ACTOR_TARGET_TYPE = T_ACTOR_TARGET_TYPE;
        using CRITIC_TYPE = T_CRITIC_TYPE;
        using CRITIC_TARGET_TYPE = T_CRITIC_TARGET_TYPE;
        using OPTIMIZER = T_OPTIMIZER;
        using PARAMETERS = T_PARAMETERS;
        static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = T_INCLUDE_FIRST_STEP_IN_TARGETS;
    };

    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
    struct ActorTrainingBuffersSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };

    template<typename T_SPEC>
    struct ActorTrainingBuffers{
        using SPEC = typename T_SPEC::SPEC;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Input>;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        static constexpr TI SEQUENCE_LENGTH = SPEC::PARAMETERS::SEQUENCE_LENGTH;
        static constexpr TI BATCH_SIZE = SPEC::PARAMETERS::ACTOR_BATCH_SIZE;
        static constexpr TI ACTOR_INPUT_DIM = get_last(typename SPEC::ACTOR_TYPE::INPUT_SHAPE{});
        static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static constexpr TI CRITIC_OBSERVATION_DIM = get_last(typename SPEC::CRITIC_TYPE::INPUT_SHAPE{}) - SPEC::ENVIRONMENT::ACTION_DIM;

        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> state_action_value_input;
        template<typename SPEC::TI DIM>
        using STATE_ACTION_VALUE_VIEW = typename decltype(state_action_value_input)::template VIEW_RANGE<tensor::ViewSpec<2, DIM>>;
        STATE_ACTION_VALUE_VIEW<CRITIC_OBSERVATION_DIM> observations;
        STATE_ACTION_VALUE_VIEW<ACTION_DIM> actions;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> state_action_value;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> d_output;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> d_critic_input;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>, DYNAMIC_ALLOCATION>> d_actor_output;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTOR_INPUT_DIM>, DYNAMIC_ALLOCATION>> d_actor_input;
    };
    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
    struct CriticTrainingBuffersSpecification{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };
    template<typename T_SPEC>
    struct CriticTrainingBuffers{
        using SPEC = typename T_SPEC::SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        static constexpr TI SEQUENCE_LENGTH = SPEC::PARAMETERS::SEQUENCE_LENGTH;
        static constexpr TI NEXT_SEQUENCE_LENGTH = SPEC::INCLUDE_FIRST_STEP_IN_TARGETS ? SEQUENCE_LENGTH + 1 : SEQUENCE_LENGTH;
        static constexpr TI BATCH_SIZE = SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
        static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static constexpr TI CRITIC_OBSERVATION_DIM = get_last(typename SPEC::CRITIC_TYPE::INPUT_SHAPE{}) - SPEC::ENVIRONMENT::ACTION_DIM;

        using T_INPUT = typename TYPE_POLICY::template GET<numeric_types::categories::Input>;

        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>, DYNAMIC_ALLOCATION>> target_next_action_noise;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> next_state_action_value_input;
        template<typename SPEC::TI DIM>
        using NEXT_STATE_ACTION_VALUE_VIEW = typename decltype(next_state_action_value_input)::template VIEW_RANGE<tensor::ViewSpec<2, DIM>>;
        NEXT_STATE_ACTION_VALUE_VIEW<CRITIC_OBSERVATION_DIM> next_observations;
        NEXT_STATE_ACTION_VALUE_VIEW<ACTION_DIM> next_actions;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> action_value;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> target_action_value;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_state_action_value_critic_1;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_state_action_value_critic_2;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> d_input;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> d_output;
    };

    template<typename T_SPEC>
    struct ActorCritic {
        using SPEC = T_SPEC;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;

        T target_next_action_noise_std = SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_STD;
        T target_next_action_noise_clip = SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_CLIP;
        T gamma = SPEC::PARAMETERS::GAMMA;

        typename SPEC::ACTOR_TYPE actor;
        typename SPEC::ACTOR_TARGET_TYPE actor_target;

        typename SPEC::CRITIC_TYPE critics[2];
        typename SPEC::CRITIC_TARGET_TYPE critics_target[2];

        typename SPEC::OPTIMIZER actor_optimizer;
        typename SPEC::OPTIMIZER critic_optimizers[2];
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END



#endif