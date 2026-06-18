#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OFF_POLICY_RUNNER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OFF_POLICY_RUNNER_H

// Please include the file containing the environments operations before including this file
#include "../../../rl/components/replay_buffer/replay_buffer.h"
#include "../../../utils/generic/tuple/tuple.h"


/* requirements
- Multiple environments
- Batched action inference


*/

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components::off_policy_runner {
    template <typename TI, TI T_NUM_THREADS>
    struct ExecutionHints{
        static constexpr TI NUM_THREADS = T_NUM_THREADS;
    };
    template <typename T_TYPE_POLICY, typename T_TI>
    struct ParametersDefault{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr TI N_ENVIRONMENTS = 1;
        static constexpr bool ASYMMETRIC_OBSERVATIONS = false;
        static constexpr TI REPLAY_BUFFER_CAPACITY = 10000;
        static constexpr TI EPISODE_STEP_LIMIT = 1000;
        static constexpr bool COLLECT_EPISODE_STATS = false;
        static constexpr TI EPISODE_STATS_BUFFER_SIZE = 0;
        static constexpr bool SAMPLE_PARAMETERS = true;
    };
    template<typename T_TYPE_POLICY, typename T_TI, typename T_ENVIRONMENT, typename T_POLICIES, typename T_PARAMETERS, bool T_DYNAMIC_ALLOCATION=true>
    struct Specification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        using ENVIRONMENT =  T_ENVIRONMENT;
        static constexpr TI MAX_EPISODE_LENGTH = ENVIRONMENT::EPISODE_STEP_LIMIT;
        using POLICIES = T_POLICIES;
        using PARAMETERS = T_PARAMETERS;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
#ifndef _MSC_VER
        static constexpr bool DYNAMIC_ALLOCATION_REPLAY_BUFFER = false;
        static constexpr bool DYNAMIC_ALLOCATION_EPISODE_STATS = false;
#else
#ifdef RL_TOOLS_BACKEND_ENABLE_CUDA
#fatal "CUDA requires static allocation of the replay buffers in the OffPolicyRunner"
#endif
        static constexpr bool DYNAMIC_ALLOCATION_REPLAY_BUFFER = true;
        static constexpr bool DYNAMIC_ALLOCATION_EPISODE_STATS = true;
#endif
        static_assert((PARAMETERS::ASYMMETRIC_OBSERVATIONS && ENVIRONMENT::ObservationPrivileged::DIM > 0) == PARAMETERS::ASYMMETRIC_OBSERVATIONS, "ASYMMETRIC_OBSERVATIONS requested but not available in the environment");
        static constexpr TI OBSERVATION_DIM_PRIVILEGED = PARAMETERS::ASYMMETRIC_OBSERVATIONS ? ENVIRONMENT::ObservationPrivileged::DIM : ENVIRONMENT::Observation::DIM;
        static constexpr TI OBSERVATION_DIM_PRIVILEGED_ACTUAL = PARAMETERS::ASYMMETRIC_OBSERVATIONS ? ENVIRONMENT::ObservationPrivileged::DIM : 0;
    };

    template<typename SPEC, bool DYNAMIC_ALLOCATION = SPEC::DYNAMIC_ALLOCATION>
    struct Buffers{
        // todo: make the buffer exploit the observation = observation_priviliged to save memory in the case of symmetric observations
        using T_INPUT = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Input>;
        using TI = typename SPEC::TI;

        Matrix<matrix::Specification<T_INPUT, TI, SPEC::PARAMETERS::N_ENVIRONMENTS, SPEC::ENVIRONMENT::Observation::DIM, DYNAMIC_ALLOCATION>> observations;
        using OBSERVATIONS_PRIVILEGED_STANDALONE = Matrix<matrix::Specification<T_INPUT, TI, SPEC::PARAMETERS::N_ENVIRONMENTS, SPEC::OBSERVATION_DIM_PRIVILEGED, DYNAMIC_ALLOCATION>>;
        using OBSERVATIONS_PRIVILEGED_VIEW = typename decltype(observations)::template VIEW<>;
        using OBSERVATIONS_PRIVILEGED_TYPE = rl_tools::utils::typing::conditional_t<SPEC::PARAMETERS::ASYMMETRIC_OBSERVATIONS, OBSERVATIONS_PRIVILEGED_STANDALONE, OBSERVATIONS_PRIVILEGED_VIEW>;
        OBSERVATIONS_PRIVILEGED_TYPE observations_privileged;
        Matrix<matrix::Specification<T_INPUT, TI, SPEC::PARAMETERS::N_ENVIRONMENTS, SPEC::ENVIRONMENT::ACTION_DIM, DYNAMIC_ALLOCATION>> actions;
        Matrix<matrix::Specification<T_INPUT, TI, SPEC::PARAMETERS::N_ENVIRONMENTS, SPEC::ENVIRONMENT::Observation::DIM, DYNAMIC_ALLOCATION>> next_observations;
        OBSERVATIONS_PRIVILEGED_TYPE next_observations_privileged;
    };

    // the following two base types are used to warn users when they changed the sequence length but not specify custom batch sampling parameters
    struct Dummy{};
    struct SequentialBatchParametersDefault{};
    template<typename TYPE_POLICY, typename TI, TI SEQUENCE_LENGTH, typename BASE=Dummy>
    struct SequentialBatchParameters: BASE{
        using T = typename TYPE_POLICY::DEFAULT;
        static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = SEQUENCE_LENGTH > 1; // This should be false for Markovian environments (that have SEQUENCE_LENGTH == 1). For non-Markovian environments (e.g. partial observable) this should be true, especially for environments, where the first step in the environment matters (like the flag environment, where crucial information is only revealed on the first step). In that case please also make sure to configure the batch sampling accordingly: check the other BATCH_SAMPLIN_* parameters.
        static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = SEQUENCE_LENGTH > 1;
        static constexpr bool RANDOM_SEQ_LENGTH = SEQUENCE_LENGTH > 1;
        static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.5;
    };

    template<typename T_SPEC, typename T_SPEC::TI T_SEQUENCE_LENGTH, typename T_SPEC::TI T_BATCH_SIZE, typename T_PARAMETERS = SequentialBatchParameters<typename T_SPEC::TYPE_POLICY, typename T_SPEC::TI, T_SEQUENCE_LENGTH>, bool T_DYNAMIC_ALLOCATION=true>
    struct SequentialBatchSpecification{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename T_SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI SEQUENCE_LENGTHH = T_SEQUENCE_LENGTH;
        static constexpr TI PADDED_SEQUENCE_LENGTH = SEQUENCE_LENGTHH + 1;
        static constexpr TI BATCH_SIZE = T_BATCH_SIZE;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        using PARAMETERS = T_PARAMETERS;
    };


    template <typename T_SPEC>
    struct SequentialBatch{
        using SPEC = T_SPEC;
        using OPR_SPEC = typename T_SPEC::SPEC;
        using TYPE_POLICY = typename OPR_SPEC::TYPE_POLICY;
        using T_INPUT = typename TYPE_POLICY::template GET<numeric_types::categories::Input>;
        using T_DEFAULT = typename TYPE_POLICY::DEFAULT;
        using TI = typename OPR_SPEC::TI;

        static constexpr TI BATCH_SIZE = T_SPEC::BATCH_SIZE;
        static constexpr TI SEQUENCE_LENGTHH = T_SPEC::SEQUENCE_LENGTHH;
        static constexpr TI PADDED_SEQUENCE_LENGTH = T_SPEC::PADDED_SEQUENCE_LENGTH;
        static constexpr TI OBSERVATION_DIM = OPR_SPEC::ENVIRONMENT::Observation::DIM;
        static constexpr bool ASYMMETRIC_OBSERVATIONS = OPR_SPEC::PARAMETERS::ASYMMETRIC_OBSERVATIONS;
        static constexpr TI OBSERVATION_DIM_PRIVILEGED = OPR_SPEC::OBSERVATION_DIM_PRIVILEGED;
        static constexpr TI ACTION_DIM = OPR_SPEC::ENVIRONMENT::ACTION_DIM;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;

        static constexpr TI DATA_DIM = OBSERVATION_DIM + OPR_SPEC::OBSERVATION_DIM_PRIVILEGED_ACTUAL + ACTION_DIM;
        Tensor<tensor::Specification<T_INPUT, TI, tensor::Shape<TI, PADDED_SEQUENCE_LENGTH, BATCH_SIZE, DATA_DIM>, DYNAMIC_ALLOCATION>> observations_actions_base;

        template<typename OPR_SPEC::TI DIM>
        using OA_VIEW_BASE = typename decltype(observations_actions_base)::template VIEW_RANGE<tensor::ViewSpec<2, DIM>>;

        template<typename BASE>
        using OA_VIEW = typename BASE::template VIEW_RANGE<tensor::ViewSpec<0, SEQUENCE_LENGTHH>>;

        static constexpr TI TARGET_SEQUENCE_LENGTH = T_SPEC::PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? PADDED_SEQUENCE_LENGTH : SEQUENCE_LENGTHH;
        template<typename BASE>
        using OA_VIEW_NEXT = typename BASE::template VIEW_RANGE<tensor::ViewSpec<0, TARGET_SEQUENCE_LENGTH>>;

        OA_VIEW_BASE<OBSERVATION_DIM> observations_base;
        OA_VIEW_BASE<OPR_SPEC::OBSERVATION_DIM_PRIVILEGED> observations_privileged_base;
        OA_VIEW_BASE<ACTION_DIM> actions_base;
        OA_VIEW_BASE<OPR_SPEC::OBSERVATION_DIM_PRIVILEGED + ACTION_DIM> observations_and_actions_base;

        OA_VIEW <decltype(observations_base)> observations_current;
        OA_VIEW <decltype(observations_privileged_base)> observations_privileged_current;
        OA_VIEW <decltype(actions_base)> actions_current;
        OA_VIEW <decltype(observations_and_actions_base)> observations_and_actions_current;

        OA_VIEW_NEXT<decltype(observations_base)> observations_next;
        OA_VIEW_NEXT<decltype(observations_privileged_base)> observations_privileged_next;
        OA_VIEW_NEXT<decltype(actions_base)> actions_next;

        Tensor<tensor::Specification<T_DEFAULT, TI, tensor::Shape<TI, SEQUENCE_LENGTHH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> rewards;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, SEQUENCE_LENGTHH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> terminated;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, SEQUENCE_LENGTHH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> reset;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, PADDED_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_reset_base;
        typename decltype(next_reset_base)::template VIEW_RANGE<tensor::ViewSpec<0, TARGET_SEQUENCE_LENGTH>> next_reset;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, SEQUENCE_LENGTHH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> final_step_mask;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, PADDED_SEQUENCE_LENGTH, BATCH_SIZE, 1>, DYNAMIC_ALLOCATION>> next_final_step_mask_base;
        typename decltype(next_final_step_mask_base)::template VIEW_RANGE<tensor::ViewSpec<0, TARGET_SEQUENCE_LENGTH>> next_final_step_mask;

    };

    template <typename T_T, typename T_TI, T_TI T_BUFFER_SIZE, bool T_DYNAMIC_ALLOCATION=true>
    struct EpisodeStatsSpecification{
        using T = T_T;
        using TI = T_TI;
        static constexpr TI BUFFER_SIZE = T_BUFFER_SIZE;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };

    template<typename T_SPEC>
    struct EpisodeStats{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        Matrix<matrix::Specification<T, TI, SPEC::BUFFER_SIZE, 2, SPEC::DYNAMIC_ALLOCATION>> data;

        TI next_episode_i = 0;
        template<typename SPEC::TI DIM>
        using STATS_VIEW = typename decltype(data)::template VIEW<SPEC::BUFFER_SIZE, DIM>;
        STATS_VIEW<1> returns;
        STATS_VIEW<1> steps;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components{
    template<typename T_SPEC>
    struct OffPolicyRunner{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        using ENVIRONMENT = typename SPEC::ENVIRONMENT;
        using POLICIES = typename SPEC::POLICIES;
        template <typename INPUT>
        struct GET_STATE{
            using CONTENT = typename INPUT::template State<SPEC::DYNAMIC_ALLOCATION>;
        };
        using POLICY_STATES = rl_tools::utils::MapTuple<POLICIES, GET_STATE>;
        using REPLAY_BUFFER_SPEC = replay_buffer::Specification<TYPE_POLICY, typename SPEC::TI, SPEC::ENVIRONMENT::Observation::DIM, ENVIRONMENT::ObservationPrivileged::DIM, SPEC::PARAMETERS::ASYMMETRIC_OBSERVATIONS, SPEC::ENVIRONMENT::ACTION_DIM, SPEC::PARAMETERS::REPLAY_BUFFER_CAPACITY, SPEC::DYNAMIC_ALLOCATION_REPLAY_BUFFER>;
        using REPLAY_BUFFER_WITH_STATES_SPEC = replay_buffer::SpecificationWithStates<ENVIRONMENT, REPLAY_BUFFER_SPEC>;
        using REPLAY_BUFFER_TYPE = ReplayBufferWithStates<REPLAY_BUFFER_WITH_STATES_SPEC>;
        static constexpr TI N_ENVIRONMENTS = SPEC::PARAMETERS::N_ENVIRONMENTS;

        off_policy_runner::Buffers<SPEC> buffers;

        TI previous_policy = 0;
        bool previous_policy_set = false;

        // todo: change to "environments"
        Matrix<matrix::Specification<ENVIRONMENT, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> envs;
        POLICY_STATES policy_states;
        Matrix<matrix::Specification<off_policy_runner::EpisodeStats<off_policy_runner::EpisodeStatsSpecification<T, TI, SPEC::PARAMETERS::EPISODE_STATS_BUFFER_SIZE, SPEC::DYNAMIC_ALLOCATION_EPISODE_STATS>>, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> episode_stats;
        Matrix<matrix::Specification<REPLAY_BUFFER_TYPE, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> replay_buffers = {};

        Matrix<matrix::Specification<typename SPEC::ENVIRONMENT::State, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> states;
        Matrix<matrix::Specification<typename SPEC::ENVIRONMENT::Parameters, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> env_parameters;
        Matrix<matrix::Specification<T, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> episode_return;
        Matrix<matrix::Specification<TI, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> episode_step;
        Matrix<matrix::Specification<bool, TI, 1, N_ENVIRONMENTS, SPEC::DYNAMIC_ALLOCATION>> truncated; // init to true !!
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        bool initialized = false;
#endif
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
