constexpr TI SEQUENCE_LENGTH = 10;
constexpr TI SEQUENCE_LENGTH_PROXY = SEQUENCE_LENGTH;
constexpr TI BATCH_SIZE = 64;
constexpr TI NUM_CHECKPOINTS = 100;
//struct ENVIRONMENT_PARAMETERS{
//    constexpr static TI HORIZON = 100;
//    constexpr static T INPUT_PROBABILITY = HORIZON <= 4 ? 0.5 : (T)2/HORIZON;
//    static constexpr TI EPISODE_STEP_LIMIT = 2000;
//    constexpr static rlt::rl::environments::memory::Mode MODE = rlt::rl::environments::memory::Mode::COUNT_INPUT;
//};


namespace env_param_builder{
    using namespace rlt::rl::environments::l2f;
    struct ENVIRONMENT_PARAMETERS{
        struct ENVIRONMENT_STATIC_PARAMETERS{
            static constexpr TI N_SUBSTEPS = 1;
            static constexpr TI ACTION_HISTORY_LENGTH = 1;
            static constexpr TI EPISODE_STEP_LIMIT = 500;
            static constexpr bool CLOSED_FORM = false;
            using STATE_BASE = StateLinearAcceleration<StateSpecification<T, TI, StateBase<StateSpecification<T, TI>>>>;
            using STATE_TYPE_NORMAL = StateRotorsHistory<StateRotorsHistorySpecification<T, TI, ACTION_HISTORY_LENGTH, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>;
            using STATE_TYPE_PARTIAL_OBSERVED = StateRotors<StateRotorsSpecification<T, TI, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>;
            using OBSERVATION_TYPE_NORMAL = observation::Position<observation::PositionSpecification<T, TI,
                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                    observation::LinearVelocity<observation::LinearVelocitySpecification<T, TI,
                    observation::AngularVelocity<observation::AngularVelocitySpecification<T, TI,
                    observation::ActionHistory<observation::ActionHistorySpecification<T, TI, ACTION_HISTORY_LENGTH>>>>>>>>>>;
            using OBSERVATION_TYPE_PRIVILEGED_NORMAL = observation::Position<observation::PositionSpecificationPrivileged<T, TI,
                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecificationPrivileged<T, TI,
                    observation::LinearVelocity<observation::LinearVelocitySpecificationPrivileged<T, TI,
                    observation::AngularVelocity<observation::AngularVelocitySpecificationPrivileged<T, TI,
                    observation::RandomForce<observation::RandomForceSpecification<T, TI,
                    observation::RotorSpeeds<observation::RotorSpeedsSpecification<T, TI>>>>>>>>>>>>;
            using OBSERVATION_TYPE_PARTIALLY_OBSERVED =
                    observation::Position<observation::PositionSpecification<T, TI,
//                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                    observation::LinearVelocity<observation::LinearVelocitySpecification<T, TI,
                    observation::AngularVelocity<observation::AngularVelocitySpecification<T, TI,
                    observation::IMUAccelerometer<observation::IMUAccelerometerSpecification<T, TI,
                    observation::Magnetometer<observation::MagnetometerSpecification<T, TI,
                    observation::ActionHistory<observation::ActionHistorySpecification<T, TI, ACTION_HISTORY_LENGTH
//                    observation::RotorSpeeds<observation::RotorSpeedsSpecification<T, TI
                    >>>>>>>>>>>>;
            using OBSERVATION_TYPE_PRIVILEGED_PARTIALLY_OBSERVED =
                    observation::Position<observation::PositionSpecificationPrivileged<T, TI,
                    observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecificationPrivileged<T, TI,
                    observation::LinearVelocity<observation::LinearVelocitySpecificationPrivileged<T, TI,
                    observation::AngularVelocity<observation::AngularVelocitySpecificationPrivileged<T, TI,
                    observation::IMUAccelerometer<observation::IMUAccelerometerSpecificationPrivileged<T, TI,
                    observation::RandomForce<observation::RandomForceSpecification<T, TI,
                    observation::RotorSpeeds<observation::RotorSpeedsSpecification<T, TI>>>>>>>>>>>>>>;
            using OBSERVATION_TYPE = OBSERVATION_TYPE_PARTIALLY_OBSERVED;
            using OBSERVATION_TYPE_PRIVILEGED = OBSERVATION_TYPE_PRIVILEGED_PARTIALLY_OBSERVED;
            using STATE_TYPE = STATE_TYPE_NORMAL;
//            using OBSERVATION_TYPE = OBSERVATION_TYPE_NORMAL;
//            using OBSERVATION_TYPE_PRIVILEGED = OBSERVATION_TYPE_PRIVILEGED_NORMAL;
//            using STATE_TYPE = STATE_TYPE_NORMAL;
            static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
            using BASE = rl_tools::rl::environments::l2f::parameters::DEFAULT_PARAMETERS_FACTORY<T, TI>;
            using PARAMETERS = typename BASE::PARAMETERS_TYPE;
            static constexpr auto PARAMETER_VALUES = BASE::nominal_parameters;
            static constexpr T STATE_LIMIT_POSITION = 100000;
            static constexpr T STATE_LIMIT_VELOCITY = 100000;
            static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
        };

        using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI, ENVIRONMENT_STATIC_PARAMETERS>;
        using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;
    };
}

using ENVIRONMENT = typename env_param_builder::ENVIRONMENT_PARAMETERS::ENVIRONMENT;
//using ENVIRONMENT_SPEC = rlt::rl::environments::memory::Specification<T, TI, ENVIRONMENT_PARAMETERS>;
//using ENVIRONMENT = rlt::rl::environments::Memory<ENVIRONMENT_SPEC>;
//using ENVIRONMENT_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
//using ENVIRONMENT = rlt::rl::environments::Pendulum<ENVIRONMENT_SPEC>;

struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr T GAMMA = 0.99;
        static constexpr TI ACTOR_BATCH_SIZE = BATCH_SIZE;
        static constexpr TI CRITIC_BATCH_SIZE = BATCH_SIZE;
        static constexpr TI SEQUENCE_LENGTH = SEQUENCE_LENGTH_PROXY;
        static constexpr TI TRAINING_INTERVAL = 10;
        static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
        static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
        static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
        static constexpr bool ENTROPY_BONUS = true;
        static constexpr bool ENTROPY_BONUS_NEXT_STEP = false;
        static constexpr bool MASK_NON_TERMINAL = true;
//        static constexpr T ALPHA = 0.01;
//        static constexpr T TARGET_ENTROPY = -((T)8);

    };
    static constexpr TI N_WARMUP_STEPS = 1000;
    static constexpr TI N_WARMUP_STEPS_CRITIC = 1000;
    static constexpr TI N_WARMUP_STEPS_ACTOR = 1000;
    static constexpr TI STEP_LIMIT = 10000000;
    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
    static constexpr TI ACTOR_HIDDEN_DIM = 32;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::TANH;
    static constexpr TI CRITIC_HIDDEN_DIM = ACTOR_HIDDEN_DIM;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = ACTOR_ACTIVATION_FUNCTION;

    struct BATCH_SAMPLING_PARAMETERS{
        static constexpr bool INCLUDE_FIRST_STEP_IN_TARGETS = true;
        static constexpr bool ALWAYS_SAMPLE_FROM_INITIAL_STATE = false;
        static constexpr bool RANDOM_SEQ_LENGTH = true;
        static constexpr bool ENABLE_NOMINAL_SEQUENCE_LENGTH_PROBABILITY = true;
        static constexpr T NOMINAL_SEQUENCE_LENGTH_PROBABILITY = 0.5;
    };

    struct ACTOR_OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 1e-3;
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
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else

// template<typename T, typename TI, typename ENVIRONMENT, typename PARAMETERS>
// using ConfigApproximatorsSequentialBoundSequenceLength = ConfigApproximatorsSequential<T, TI, SEQUENCE_LENGTH, ENVIRONMENT, PARAMETERS>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsGRU>;
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_CORE_CONFIG>;
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_EXTRACK_CONFIG>{
    static constexpr TI EVALUATION_INTERVAL = 10000;
    static constexpr TI NUM_EVALUATION_EPISODES = 10;
//    static constexpr TI EPISODE_STEP_LIMIT = 200;
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
