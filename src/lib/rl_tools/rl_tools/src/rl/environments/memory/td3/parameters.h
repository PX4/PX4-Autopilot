constexpr bool MEMORY = true;
constexpr bool MEMORY_LONG = false;

constexpr TI SEQUENCE_LENGTH = MEMORY ? (MEMORY_LONG ? 500 : 50) : 10;
constexpr TI SEQUENCE_LENGTH_PROXY = SEQUENCE_LENGTH;
constexpr TI BATCH_SIZE = MEMORY ? 4: 100;
constexpr TI NUM_CHECKPOINTS = 100;
struct ENVIRONMENT_PARAMETERS{
    constexpr static TI HORIZON = MEMORY_LONG ? 100 : 10;
    constexpr static T INPUT_PROBABILITY = HORIZON <= 4 ? 0.5 : (T)2/HORIZON;
    static constexpr TI EPISODE_STEP_LIMIT = 2000;
    constexpr static rlt::rl::environments::memory::Mode MODE = rlt::rl::environments::memory::Mode::COUNT_INPUT;
};
using MEMORY_ENVIRONMENT_SPEC = rlt::rl::environments::memory::Specification<T, TI, ENVIRONMENT_PARAMETERS>;
using MEMORY_ENVIRONMENT = rlt::rl::environments::Memory<MEMORY_ENVIRONMENT_SPEC>;
using PENDULUM_ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using PENDULUM_ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_ENVIRONMENT_SPEC>;

using ENVIRONMENT = rlt::utils::typing::conditional_t<MEMORY, MEMORY_ENVIRONMENT, PENDULUM_ENVIRONMENT>;


struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::td3::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct TD3_PARAMETERS: rlt::rl::algorithms::td3::DefaultParameters<TYPE_POLICY, TI>{
        static constexpr T GAMMA = MEMORY ? 0.0 : 0.99;
        static constexpr TI ACTOR_BATCH_SIZE = BATCH_SIZE;
        static constexpr TI CRITIC_BATCH_SIZE = BATCH_SIZE;
        static constexpr TI SEQUENCE_LENGTH = SEQUENCE_LENGTH_PROXY;
        static constexpr TI CRITIC_TRAINING_INTERVAL = 1;
        static constexpr TI ACTOR_TRAINING_INTERVAL = 2;
        static constexpr bool ENTROPY_BONUS = true;
        static constexpr bool ENTROPY_BONUS_NEXT_STEP = false;

        static constexpr T TARGET_ENTROPY = MEMORY ? -4 : -1;
        static constexpr T ALPHA = 1;
        static constexpr bool ADAPTIVE_ALPHA = true;
    };
    static constexpr TI N_WARMUP_STEPS = 1000;
    static constexpr TI N_WARMUP_STEPS_CRITIC = 1000;
    static constexpr TI N_WARMUP_STEPS_ACTOR = MEMORY ? 10000: 1000;
    static constexpr TI STEP_LIMIT = 200000;
    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
    static constexpr TI ACTOR_HIDDEN_DIM = MEMORY ? (MEMORY_LONG ? 64 : 16) : 32;
    static constexpr TI ACTOR_NUM_LAYERS = 4;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::TANH;
    static constexpr TI CRITIC_HIDDEN_DIM = ACTOR_HIDDEN_DIM;
    static constexpr TI CRITIC_NUM_LAYERS = 4;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = ACTOR_ACTIVATION_FUNCTION;
    static constexpr bool SHARED_BATCH = false;
    static constexpr TI N_ENVIRONMENTS = 1;

    struct BATCH_SAMPLING_PARAMETERS{
        static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = false;
        static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
        static constexpr bool RANDOM_SEQ_LENGTH = true;
        static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.5;
    };

    struct ACTOR_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 1e-4;
        static constexpr bool ENABLE_BIAS_LR_FACTOR = false;
        static constexpr T BIAS_LR_FACTOR = 1;
    };
    struct CRITIC_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 1e-3;
        static constexpr bool ENABLE_BIAS_LR_FACTOR = false;
        static constexpr T BIAS_LR_FACTOR = 1;
    };
    struct ALPHA_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 1e-3;
        static constexpr bool ENABLE_BIAS_LR_FACTOR = false;
        static constexpr T BIAS_LR_FACTOR = 1;
    };
};
#ifdef BENCHMARK
using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else

using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using LOOP_CORE_CONFIG = rlt::rl::algorithms::td3::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::td3::loop::core::ConfigApproximatorsGRU>;
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_CORE_CONFIG>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_EXTRACK_CONFIG>{
    static constexpr TI EVALUATION_INTERVAL = 1000;
    static constexpr TI NUM_EVALUATION_EPISODES = 10;
    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
};
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_EXTRACK_CONFIG, LOOP_EVAL_PARAMETERS>;
struct LOOP_CHECKPOINT_PARAMETERS: rlt::rl::loop::steps::checkpoint::Parameters<TYPE_POLICY, TI>{
    static constexpr TI CHECKPOINT_INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_CHECKPOINTS;
    static constexpr TI CHECKPOINT_INTERVAL = CHECKPOINT_INTERVAL_TEMP == 0 ? 1 : CHECKPOINT_INTERVAL_TEMP;
};
using LOOP_CHECKPOINT_CONFIG = rlt::rl::loop::steps::checkpoint::Config<LOOP_EVAL_CONFIG, LOOP_CHECKPOINT_PARAMETERS>;
struct LOOP_SAVE_TRAJECTORIES_PARAMETERS: rlt::rl::loop::steps::save_trajectories::Parameters<TYPE_POLICY, TI, LOOP_CHECKPOINT_CONFIG>{
    static constexpr TI INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / 10;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
    static constexpr TI NUM_EPISODES = 10;
};
using LOOP_SAVE_TRAJECTORIES_CONFIG = rlt::rl::loop::steps::save_trajectories::Config<LOOP_CHECKPOINT_CONFIG, LOOP_SAVE_TRAJECTORIES_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_SAVE_TRAJECTORIES_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
//using LOOP_CONFIG = LOOP_EXTRACK_CONFIG;
#endif
