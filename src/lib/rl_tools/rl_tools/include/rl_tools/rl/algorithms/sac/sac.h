#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_SAC_SAC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_SAC_SAC_H
//#include "../../../nn_models/output_view/model.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::sac {
    template<typename TYPE_POLICY, typename TI, TI ACTION_DIM=1>
    struct DefaultParameters {
        using T = typename TYPE_POLICY::DEFAULT;
        static constexpr T GAMMA = 0.99;
        static constexpr TI ACTOR_BATCH_SIZE = 32;
        static constexpr TI CRITIC_BATCH_SIZE = 32;
        static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
        static constexpr TI ACTOR_TRAINING_INTERVAL = 1;
        static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1;
        static constexpr T ACTOR_POLYAK = 1.0 - 0.005;
        static constexpr T CRITIC_POLYAK = 1.0 - 0.005;
        static constexpr bool IGNORE_TERMINATION = false; // ignoring the termination flag is useful for training on environments with negative rewards, where the agent would try to terminate the episode as soon as possible otherwise
        static constexpr TI SEQUENCE_LENGTH = 1; // note that this implementation does only show next_observation sequences to the target actor and critic. Hence they have one step (the initial one in the sequence) less information. This makes the sequence length deterministic (otherwise it would depend on the number of resets in the batch). For most environments and for larger sequences the information gain should be negligible but for some (mostly artifiical) environments the first state matters (e.g. the FlagMemory environment). A possible mitigations is repeating the initial observation in the environment
        static constexpr bool ENTROPY_BONUS = true;
        static constexpr bool ENTROPY_BONUS_NEXT_STEP = true;
        static constexpr bool MASK_NON_TERMINAL = true;

        static constexpr T TARGET_ENTROPY = -((T)ACTION_DIM);
        static constexpr T ALPHA = 0.5;
        static constexpr bool ADAPTIVE_ALPHA = true;
        static constexpr T LOG_STD_LOWER_BOUND = -20;
        static constexpr T LOG_STD_UPPER_BOUND = 2;
        static constexpr T LOG_PROBABILITY_EPSILON = 1e-6;
    };

    template<
        typename T_TYPE_POLICY,
        typename T_TI,
        typename T_ENVIRONMENT,
        typename T_ACTOR_NETWORK_TYPE,
        typename T_CRITIC_NETWORK_TYPE,
        typename T_CRITIC_TARGET_NETWORK_TYPE,
        typename T_ALPHA_PARAMETER_TYPE,
        typename T_ACTOR_OPTIMIZER,
        typename T_CRITIC_OPTIMIZER,
        typename T_ALPHA_OPTIMIZER,
        typename T_PARAMETERS,
        bool T_INCLUDE_FIRST_STEP_IN_TARGETS
    >
    struct Specification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        using ENVIRONMENT = T_ENVIRONMENT;
        using ACTOR_NETWORK_TYPE = T_ACTOR_NETWORK_TYPE;
        using CRITIC_NETWORK_TYPE = T_CRITIC_NETWORK_TYPE;
        using CRITIC_TARGET_NETWORK_TYPE = T_CRITIC_TARGET_NETWORK_TYPE;
        using ALPHA_PARAMETER_TYPE = T_ALPHA_PARAMETER_TYPE;
        using ACTOR_OPTIMIZER = T_ACTOR_OPTIMIZER;
        using CRITIC_OPTIMIZER = T_CRITIC_OPTIMIZER;
        using ALPHA_OPTIMIZER = T_ALPHA_OPTIMIZER;
        using PARAMETERS = T_PARAMETERS;
        static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = T_INCLUDE_FIRST_STEP_IN_TARGETS;
    };

    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
    struct ActorTrainingBuffersSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };

    template<typename T_SPEC>
    struct ActorTrainingBuffers{
        using SPEC = typename T_SPEC::SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using T = typename TYPE_POLICY::template GET<numeric_types::categories::Buffer>;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        static constexpr TI SEQUENCE_LENGTH = SPEC::PARAMETERS::SEQUENCE_LENGTH;
        static constexpr TI BATCH_SIZE = SPEC::PARAMETERS::ACTOR_BATCH_SIZE;
        static constexpr TI ACTOR_INPUT_DIM = get_last(typename SPEC::ACTOR_NETWORK_TYPE::INPUT_SHAPE{});
        static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static constexpr TI CRITIC_OBSERVATION_DIM = get_last(typename SPEC::CRITIC_NETWORK_TYPE::INPUT_SHAPE{}) - SPEC::ENVIRONMENT::ACTION_DIM;

        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> state_action_value_input;
        template<typename SPEC::TI DIM>
        using STATE_ACTION_VALUE_VIEW = typename decltype(state_action_value_input)::template VIEW_RANGE<tensor::ViewSpec<2, DIM>>;
        STATE_ACTION_VALUE_VIEW<CRITIC_OBSERVATION_DIM> observations;
        STATE_ACTION_VALUE_VIEW<ACTION_DIM> actions;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> d_output;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> d_critic_1_input, d_critic_2_input;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>, DYNAMIC_ALLOCATION>> d_critic_action_input;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>, DYNAMIC_ALLOCATION>> action_sample, action_noise;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>, DYNAMIC_ALLOCATION>> d_actor_output_squashing;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM * 2>, DYNAMIC_ALLOCATION>> d_squashing_input;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM * 2>, DYNAMIC_ALLOCATION>> d_actor_output;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTOR_INPUT_DIM>, DYNAMIC_ALLOCATION>> d_actor_input;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, 1>, DYNAMIC_ALLOCATION>> loss_weight;
    };
    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
    struct CriticTrainingBuffersSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };
    template<typename T_SPEC>
    struct CriticTrainingBuffers{
        using SPEC = typename T_SPEC::SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using T_BUFFER = typename TYPE_POLICY::template GET<numeric_types::categories::Buffer>;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        static constexpr TI SEQUENCE_LENGTH = SPEC::PARAMETERS::SEQUENCE_LENGTH;
        static constexpr TI NEXT_SEQUENCE_LENGTH = SPEC::INCLUDE_FIRST_STEP_IN_TARGETS ? SEQUENCE_LENGTH + 1 : SEQUENCE_LENGTH;
        static constexpr TI BATCH_SIZE = SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
        static constexpr TI ACTION_DIM = SPEC::ENVIRONMENT::ACTION_DIM;
        static constexpr TI CRITIC_OBSERVATION_DIM = get_last(typename SPEC::CRITIC_NETWORK_TYPE::INPUT_SHAPE{}) - SPEC::ENVIRONMENT::ACTION_DIM;


        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> next_state_action_value_input;
        template<typename SPEC::TI DIM>
        using NEXT_STATE_ACTION_VALUE_VIEW = typename decltype(next_state_action_value_input)::template VIEW_RANGE<tensor::ViewSpec<2, DIM>>;
        NEXT_STATE_ACTION_VALUE_VIEW<CRITIC_OBSERVATION_DIM> next_observations;
        NEXT_STATE_ACTION_VALUE_VIEW<ACTION_DIM> next_actions;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> action_value;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> target_action_value;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_state_action_value_critic_1;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_state_action_value_critic_2;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, CRITIC_OBSERVATION_DIM + ACTION_DIM>, DYNAMIC_ALLOCATION>> d_input;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> d_output;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, 1>, DYNAMIC_ALLOCATION>> loss_weight;
        Tensor<tensor::Specification<T_BUFFER, TI, tensor::Shape<TI, NEXT_SEQUENCE_LENGTH * BATCH_SIZE>, DYNAMIC_ALLOCATION>> next_action_log_probs;
    };

    template<typename T_SPEC>
    struct ActorCritic {
        using SPEC = T_SPEC;
        // using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        typename SPEC::ACTOR_NETWORK_TYPE actor;
        typename SPEC::CRITIC_NETWORK_TYPE critics[2];
        typename SPEC::CRITIC_TARGET_NETWORK_TYPE critics_target[2];

        typename SPEC::ACTOR_OPTIMIZER actor_optimizer;
        typename SPEC::CRITIC_OPTIMIZER critic_optimizers[2];
        typename SPEC::ALPHA_OPTIMIZER alpha_optimizer;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END



#endif