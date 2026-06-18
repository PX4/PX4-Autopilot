#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ALGORITHMS_SAC_LOOP_CORE_CONFIG_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ALGORITHMS_SAC_LOOP_CORE_CONFIG_H

#include "../../../../../nn_models/sequential/model.h"
#include "../../../../../nn_models/mlp/network.h"
#include "../../../../../nn_models/random_uniform/model.h"
#include "../../../../../rl/algorithms/sac/sac.h"
#include "../../../../../nn/optimizers/adam/adam.h"
#include "state.h"
#include "approximators_mlp.h"
#include "approximators_gru.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::algorithms::sac::loop::core{
    // Config State (Init/Step)
    template<typename TYPE_POLICY, typename TI, typename ENVIRONMENT>
    struct DefaultParameters{
        using SAC_PARAMETERS = rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>;
        static constexpr TI N_ENVIRONMENTS = 1;
        static constexpr TI N_WARMUP_STEPS = 100; // Exploration executed with a uniform random policy for N_WARMUP_STEPS steps
        static constexpr TI N_WARMUP_STEPS_CRITIC = 100; // Number of steps before critic training starts
        static constexpr TI N_WARMUP_STEPS_ACTOR = 100; // Number of steps before actor training starts
        static_assert(N_WARMUP_STEPS >= SAC_PARAMETERS::ACTOR_BATCH_SIZE);
        static constexpr TI STEP_LIMIT = 10000;
        static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT; // Note: when inheriting from this class for overwriting the default STEP_LIMIT you need to set the REPLAY_BUFFER_CAP as well otherwise it will be the default step limit
        static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;

        static constexpr TI ACTOR_HIDDEN_DIM = 64;
        static constexpr TI ACTOR_NUM_LAYERS = 3;
        static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
        static constexpr TI CRITIC_HIDDEN_DIM = 64;
        static constexpr TI CRITIC_NUM_LAYERS = 3;
        static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;

        static constexpr bool COLLECT_EPISODE_STATS = true;
        static constexpr TI EPISODE_STATS_BUFFER_SIZE = 1000;

        static constexpr bool SHARED_BATCH = true;
        static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = true;

        using BATCH_SAMPLING_PARAMETERS = rl::components::off_policy_runner::SequentialBatchParameters<TYPE_POLICY, TI, SAC_PARAMETERS::SEQUENCE_LENGTH, rl::components::off_policy_runner::SequentialBatchParametersDefault>;

        using INITIALIZER = nn::layers::dense::DefaultInitializer<TYPE_POLICY, TI>;

        using ACTOR_OPTIMIZER_PARAMETERS = nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>;
        using CRITIC_OPTIMIZER_PARAMETERS = nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>;
        using ALPHA_OPTIMIZER_PARAMETERS = nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>;
    };


    template<typename T_TYPE_POLICY, typename T_TI, typename T_RNG, typename T_ENVIRONMENT, typename T_PARAMETERS = DefaultParameters<T_TYPE_POLICY, T_TI, T_ENVIRONMENT>, template<typename, typename, typename, typename, bool> class APPROXIMATOR_CONFIG=ConfigApproximatorsMLP, bool T_DYNAMIC_ALLOCATION=true>
    struct Config{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        using RNG = T_RNG;
        using ENVIRONMENT = T_ENVIRONMENT;
        using ENVIRONMENT_EVALUATION = T_ENVIRONMENT;
        using CORE_PARAMETERS = T_PARAMETERS;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;

        static constexpr TI ENVIRONMENT_STEPS_PER_LOOP_STEP = CORE_PARAMETERS::N_ENVIRONMENTS;

        using NN = APPROXIMATOR_CONFIG<TYPE_POLICY, TI, T_ENVIRONMENT, CORE_PARAMETERS, DYNAMIC_ALLOCATION>;

        using EXPLORATION_POLICY_SPEC = nn_models::random_uniform::Specification<TYPE_POLICY, TI, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, nn_models::random_uniform::Range::MINUS_ONE_TO_ONE>;
        using EXPLORATION_POLICY = nn_models::RandomUniform<EXPLORATION_POLICY_SPEC>;

        using ALPHA_PARAMETER_TYPE = nn::parameters::Adam;
//        using ALPHA_OPTIMIZER = nn::optimizers::Adam<typename NN::ALPHA_OPTIMIZER_SPEC>;

        using ACTOR_CRITIC_SPEC = rl::algorithms::sac::Specification<TYPE_POLICY, TI, ENVIRONMENT, typename NN::ACTOR_TYPE, typename NN::CRITIC_TYPE, typename NN::CRITIC_TARGET_TYPE, ALPHA_PARAMETER_TYPE, typename NN::ACTOR_OPTIMIZER, typename NN::CRITIC_OPTIMIZER, typename NN::ALPHA_OPTIMIZER, typename CORE_PARAMETERS::SAC_PARAMETERS, CORE_PARAMETERS::BATCH_SAMPLING_PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS>;
        using ACTOR_CRITIC_TYPE = rl::algorithms::sac::ActorCritic<ACTOR_CRITIC_SPEC>;
        static constexpr TI NUM_NNS = 3;
        using EVAL_ACTOR_TYPE = typename NN::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, CORE_PARAMETERS::N_ENVIRONMENTS>;

        struct OFF_POLICY_RUNNER_PARAMETERS{
            static constexpr TI N_ENVIRONMENTS = CORE_PARAMETERS::N_ENVIRONMENTS;
            static constexpr bool ASYMMETRIC_OBSERVATIONS = !rl_tools::utils::typing::is_same_v<typename ENVIRONMENT::Observation, typename ENVIRONMENT::ObservationPrivileged>;
            static constexpr TI REPLAY_BUFFER_CAPACITY = CORE_PARAMETERS::REPLAY_BUFFER_CAP;
            static constexpr TI EPISODE_STEP_LIMIT = CORE_PARAMETERS::EPISODE_STEP_LIMIT;
            static constexpr bool COLLECT_EPISODE_STATS = CORE_PARAMETERS::COLLECT_EPISODE_STATS;
            static constexpr TI EPISODE_STATS_BUFFER_SIZE = CORE_PARAMETERS::EPISODE_STATS_BUFFER_SIZE;
            static constexpr bool SAMPLE_PARAMETERS = CORE_PARAMETERS::SAMPLE_ENVIRONMENT_PARAMETERS;
        };
        using POLICIES = rl_tools::utils::Tuple<TI, EXPLORATION_POLICY, EVAL_ACTOR_TYPE>;

        using OFF_POLICY_RUNNER_SPEC = rl::components::off_policy_runner::Specification<TYPE_POLICY, TI, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS, DYNAMIC_ALLOCATION>;
        static_assert(ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE == ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE);
        static constexpr TI TARGET_SEQUENCE_LENGTH = CORE_PARAMETERS::SAC_PARAMETERS::SEQUENCE_LENGTH + (CORE_PARAMETERS::BATCH_SAMPLING_PARAMETERS::INCLUDE_FIRST_STEP_IN_TARGETS ? 1 : 0);

        static constexpr TI DEFAULT_SEQUENCE_LENGTH = DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>::SAC_PARAMETERS::SEQUENCE_LENGTH;
        static constexpr bool USING_DEFAULT_BATCH_SAMPLING_PARAMETERS = rl_tools::utils::typing::is_base_of_v<components::off_policy_runner::SequentialBatchParametersDefault, typename CORE_PARAMETERS::BATCH_SAMPLING_PARAMETERS>;
        static_assert(CORE_PARAMETERS::SAC_PARAMETERS::SEQUENCE_LENGTH == DEFAULT_SEQUENCE_LENGTH || !USING_DEFAULT_BATCH_SAMPLING_PARAMETERS, "When setting a custom sequence length, please study and set custom batch sampling parameters.");

        using CRITIC_BATCH_SPEC = rl::components::off_policy_runner::SequentialBatchSpecification<OFF_POLICY_RUNNER_SPEC, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::SEQUENCE_LENGTH, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE, typename CORE_PARAMETERS::BATCH_SAMPLING_PARAMETERS, DYNAMIC_ALLOCATION>;
        using ACTOR_BATCH_SPEC = rl::components::off_policy_runner::SequentialBatchSpecification<OFF_POLICY_RUNNER_SPEC, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::SEQUENCE_LENGTH, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE, typename CORE_PARAMETERS::BATCH_SAMPLING_PARAMETERS, DYNAMIC_ALLOCATION>;
        template <typename CONFIG>
        using State = State<CONFIG>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

