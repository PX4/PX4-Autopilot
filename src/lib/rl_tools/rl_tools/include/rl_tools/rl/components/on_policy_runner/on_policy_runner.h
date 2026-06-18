#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_ON_POLICY_RUNNER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_ON_POLICY_RUNNER_ON_POLICY_RUNNER_H

#include "../../../utils/generic/typing.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::components{
    namespace on_policy_runner{
        template <typename T_TYPE_POLICY, typename T_TI, typename T_ENVIRONMENT, T_TI T_N_ENVIRONMENTS = 1, T_TI T_STEP_LIMIT = 0, T_TI T_N_AGENTS_PER_ENV = 1, bool T_DYNAMIC_ALLOCATION=true>
        struct Specification{
            using TYPE_POLICY = T_TYPE_POLICY;
            using TI = T_TI;
            using ENVIRONMENT = T_ENVIRONMENT;
            static constexpr TI N_ENVIRONMENTS = T_N_ENVIRONMENTS;
            static constexpr TI STEP_LIMIT = T_STEP_LIMIT;
            static constexpr bool ASYMMETRIC_OBSERVATIONS = !rl_tools::utils::typing::is_same_v<typename ENVIRONMENT::Observation, typename ENVIRONMENT::ObservationPrivileged>;
            static constexpr TI N_AGENTS_PER_ENV = T_N_AGENTS_PER_ENV; // 1 for single agent, >1 for multi-agent
            static constexpr bool DYANMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };

        template <typename T_SPEC, typename T_SPEC::TI T_STEPS_PER_ENV, bool T_DYNAMIC_ALLOCATION = true>
        struct DatasetSpecification{
            using SPEC = T_SPEC;
            using TI = typename SPEC::TI;
            static constexpr TI STEPS_PER_ENV = T_STEPS_PER_ENV;
            static constexpr bool ASYMMETRIC_OBSERVATIONS = SPEC::ASYMMETRIC_OBSERVATIONS;
            static constexpr TI STEPS_TOTAL = STEPS_PER_ENV * SPEC::N_ENVIRONMENTS;
            static constexpr TI STEPS_TOTAL_ALL = (STEPS_PER_ENV+1) * SPEC::N_ENVIRONMENTS; // +1 for the final observation
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };

        template <typename T_DATASET_SPEC>
        struct Dataset{
            using DATASET_SPEC = T_DATASET_SPEC;
            using SPEC = typename DATASET_SPEC::SPEC;
            using TYPE_POLICY = typename SPEC::TYPE_POLICY;
            using T = typename TYPE_POLICY::DEFAULT;
            using TI = typename SPEC::TI;
            static constexpr TI STEPS_PER_ENV = DATASET_SPEC::STEPS_PER_ENV;
            static constexpr TI STEPS_TOTAL = DATASET_SPEC::STEPS_TOTAL;
            // structure: OBSERVATION_PRIVILIGED_DIM + OBSERVATION_DIM + ACTIONS + ACTIONS_MEAN + ACTION_LOG_P + REWARD + TERMINATED + TRUNCATED + VALUE + ADVANTAGE + TARGET_VALUE
            static constexpr TI DATA_DIM = (SPEC::ASYMMETRIC_OBSERVATIONS ? SPEC::ENVIRONMENT::ObservationPrivileged::DIM : 0) + SPEC::ENVIRONMENT::Observation::DIM + SPEC::ENVIRONMENT::ACTION_DIM * 2 + 7;

            // mem
            // todo: evaluate transposing this / storing in column major order for better memory access in the single dimensional columns
            Matrix<matrix::Specification<T, TI, STEPS_TOTAL + SPEC::N_ENVIRONMENTS, DATA_DIM, DATASET_SPEC::DYNAMIC_ALLOCATION>> data; // +1 * SPEC::N_ENVIRONMENTS for the final observation

            // views
            template<TI VIEW_DIM, bool ALL = false>
            using DATA_VIEW = typename decltype(data)::template VIEW<STEPS_TOTAL + (ALL ? SPEC::N_ENVIRONMENTS : 0), VIEW_DIM>;

            DATA_VIEW<SPEC::ENVIRONMENT::ObservationPrivileged::DIM, true> all_observations_privileged;
            DATA_VIEW<SPEC::ENVIRONMENT::Observation::DIM> observations;
            DATA_VIEW<SPEC::ENVIRONMENT::ACTION_DIM> actions_mean;
            DATA_VIEW<SPEC::ENVIRONMENT::ACTION_DIM> actions;
            DATA_VIEW<1> action_log_probs;
            DATA_VIEW<1> rewards;
            DATA_VIEW<1> terminated;
            DATA_VIEW<1> truncated;
            DATA_VIEW<1, true> all_values;
            DATA_VIEW<1> values;
            DATA_VIEW<1> advantages;
            DATA_VIEW<1> target_values;
        };
        template <typename TI, TI T_NUM_THREADS>
        struct ExecutionHints{
            static constexpr TI NUM_THREADS = T_NUM_THREADS;
        };
    }

    template <typename T_SPEC>
    struct OnPolicyRunner{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;

        TI step = 0;

        Matrix<matrix::Specification<typename SPEC::ENVIRONMENT            , TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> environments;
        Matrix<matrix::Specification<typename SPEC::ENVIRONMENT::Parameters, TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> env_parameters;
        Matrix<matrix::Specification<typename SPEC::ENVIRONMENT::State     , TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> states;
        Matrix<matrix::Specification<bool                                  , TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> truncated;
        Matrix<matrix::Specification<TI                                    , TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> episode_step;
        Matrix<matrix::Specification<typename TYPE_POLICY::DEFAULT         , TI, 1, SPEC::N_ENVIRONMENTS, SPEC::DYANMIC_ALLOCATION>> episode_return;
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_ON_POLICY_RUNNER_CHECK_INIT
        bool initialized = false;
#endif
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
