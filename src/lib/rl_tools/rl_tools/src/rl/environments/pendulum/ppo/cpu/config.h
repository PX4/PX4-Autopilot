template <typename DEVICE, typename TYPE_POLICY, bool DYNAMIC_ALLOCATION>
struct CONFIG_FACTORY{
    using TI = typename DEVICE::index_t;
    using T = typename TYPE_POLICY::DEFAULT;
    using RNG = typename DEVICE::SPEC::RANDOM::template ENGINE<>;
    using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<typename TYPE_POLICY::DEFAULT, TI, rlt::rl::environments::pendulum::DefaultParameters<typename TYPE_POLICY::DEFAULT>>;
    using PRE_ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
    using SCALE_OBSERVATIONS_WRAPPER_SPEC = rlt::rl::environment_wrappers::scale_observations::Specification<TYPE_POLICY, TI>;
    using ENVIRONMENT = rlt::rl::environment_wrappers::ScaleObservations<SCALE_OBSERVATIONS_WRAPPER_SPEC, PRE_ENVIRONMENT>;

    struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
        static constexpr TI BATCH_SIZE = 256;
        static constexpr TI ACTOR_HIDDEN_DIM = 64;
        static constexpr TI CRITIC_HIDDEN_DIM = 64;
        static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 1024;
        static constexpr TI N_ENVIRONMENTS = 4;
        static constexpr TI TOTAL_STEP_LIMIT = 300000;
        static constexpr TI STEP_LIMIT = TOTAL_STEP_LIMIT/(ON_POLICY_RUNNER_STEPS_PER_ENV * N_ENVIRONMENTS) + 1;
        static constexpr TI EPISODE_STEP_LIMIT = 200;
        using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>;
        struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
            static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.0;
            static constexpr TI N_EPOCHS = 2;
            static constexpr T GAMMA = 0.9;
            static constexpr T INITIAL_ACTION_STD = 2.0;
            static constexpr bool NORMALIZE_OBSERVATIONS = true;
        };
    };
    using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequential, DYNAMIC_ALLOCATION>;
    template <typename NEXT>
    struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, NEXT>{
        static constexpr TI EVALUATION_INTERVAL = 10;
        static constexpr TI NUM_EVALUATION_EPISODES = 100;
        static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
    };

    #ifndef BENCHMARK
    using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, LOOP_EVAL_PARAMETERS<LOOP_CORE_CONFIG>>;
    using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
    #else
    using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
    #endif

};
