#include <rl_tools/rl/environments/pendulum/pendulum.h>
#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/nn_models/models.h>
#include <rl_tools/rl/algorithms/sac/sac.h>
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/components/off_policy_runner/off_policy_runner.h>

#include <rl_tools/utils/generic/typing.h>

template<typename DEVICE, typename T>
struct parameters_pendulum_0{
    using TI = typename DEVICE::index_t;
    struct env{
        using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
        using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
    };

    template <typename ENVIRONMENT>
    struct rl{
        struct LOOP_PARAMS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<T, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<T, TI, ENVIRONMENT::ACTION_DIM>{
                constexpr static TI CRITIC_BATCH_SIZE = 100;
                constexpr static TI ACTOR_BATCH_SIZE = 100;
                static constexpr TI N_WARMUP_STEPS_ACTOR = ACTOR_BATCH_SIZE;
                static constexpr TI N_WARMUP_STEPS_CRITIC = CRITIC_BATCH_SIZE;
            };

        };
        using CFG = rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsSequential<T, TI, ENVIRONMENT, LOOP_PARAMS>;
        using OPTIMIZER = typename CFG::ACTOR_OPTIMIZER;
        using ACTOR_NETWORK_TYPE = typename CFG::ACTOR_TYPE;
        using CRITIC_NETWORK_TYPE = typename CFG::CRITIC_TYPE;
        using CRITIC_TARGET_NETWORK_TYPE = typename CFG::CRITIC_TARGET_TYPE;


        using ALPHA_PARAMETER_TYPE = rlt::nn::parameters::Adam;
        using ACTOR_CRITIC_PARAMETERS = typename LOOP_PARAMS::SAC_PARAMETERS;
        using ACTOR_CRITIC_SPEC = rlt::rl::algorithms::sac::Specification<T, TI, ENVIRONMENT, ACTOR_NETWORK_TYPE, CRITIC_NETWORK_TYPE, CRITIC_TARGET_NETWORK_TYPE, ALPHA_PARAMETER_TYPE, OPTIMIZER, OPTIMIZER, OPTIMIZER, ACTOR_CRITIC_PARAMETERS>;
        using ACTOR_CRITIC_TYPE = rlt::rl::algorithms::sac::ActorCritic<ACTOR_CRITIC_SPEC>;

        struct OFF_POLICY_RUNNER_PARAMETERS{
            static constexpr TI N_ENVIRONMENTS = 1;
            static constexpr bool ASYMMETRIC_OBSERVATIONS = false;
            static constexpr TI REPLAY_BUFFER_CAPACITY = 500000;
            static constexpr TI EPISODE_STEP_LIMIT = 200;
            static constexpr bool STOCHASTIC_POLICY = true;
            static constexpr bool COLLECT_EPISODE_STATS = false;
            static constexpr TI EPISODE_STATS_BUFFER_SIZE = false;
            static constexpr T EXPLORATION_NOISE = 0.1;
        };
        using EXPLORATION_POLICY_SPEC = rlt::nn_models::random_uniform::Specification<T, TI, ENVIRONMENT::Observation::DIM, ENVIRONMENT::ACTION_DIM, rlt::nn_models::random_uniform::Range::MINUS_ONE_TO_ONE>;
        using EXPLORATION_POLICY = rlt::nn_models::RandomUniform<EXPLORATION_POLICY_SPEC>;
        using POLICIES = rl_tools::utils::Tuple<TI, EXPLORATION_POLICY, ACTOR_NETWORK_TYPE>;
        using OFF_POLICY_RUNNER_SPEC = rlt::rl::components::off_policy_runner::Specification<T, TI, ENVIRONMENT, POLICIES, OFF_POLICY_RUNNER_PARAMETERS>;
        using OFF_POLICY_RUNNER_TYPE = rlt::rl::components::OffPolicyRunner<OFF_POLICY_RUNNER_SPEC>;
        using CRITIC_BATCH_TYPE = rlt::rl::components::off_policy_runner::Batch<rlt::rl::components::off_policy_runner::BatchSpecification<OFF_POLICY_RUNNER_SPEC, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::CRITIC_BATCH_SIZE>>;
        using ACTOR_BATCH_TYPE = rlt::rl::components::off_policy_runner::Batch<rlt::rl::components::off_policy_runner::BatchSpecification<OFF_POLICY_RUNNER_SPEC, ACTOR_CRITIC_TYPE::SPEC::PARAMETERS::ACTOR_BATCH_SIZE>>;
        using CRITIC_TRAINING_BUFFERS_TYPE = rlt::rl::algorithms::sac::CriticTrainingBuffers<typename ACTOR_CRITIC_TYPE::SPEC>;
        using ACTOR_TRAINING_BUFFERS_TYPE = rlt::rl::algorithms::sac::ActorTrainingBuffers<typename ACTOR_CRITIC_TYPE::SPEC>;

    };
};

