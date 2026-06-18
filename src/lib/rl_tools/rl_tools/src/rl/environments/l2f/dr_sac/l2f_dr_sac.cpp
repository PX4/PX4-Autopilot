#ifdef BENCHMARK
#undef RL_TOOLS_ENABLE_TENSORBOARD
#endif
#include <rl_tools/operations/cpu_mux.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/td3_sampling/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/standardize/persist.h>
#include <rl_tools/nn/layers/td3_sampling/persist.h>
#include <rl_tools/nn_models/mlp/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>

#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn/layers/gru/persist_code.h>
#include <rl_tools/nn/layers/standardize/persist_code.h>
#include <rl_tools/nn/layers/sample_and_squash/persist_code.h>
#include <rl_tools/nn/layers/td3_sampling/persist_code.h>
#include <rl_tools/nn_models/mlp/persist_code.h>
#include <rl_tools/nn_models/sequential/persist_code.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist_code.h>

#include <rl_tools/rl/environments/l2f/operations_cpu.h>
#include <rl_tools/rl/environments/l2f/operations_helper_generic.h>
#include <rl_tools/rl/environments/l2f/parameters/default.h>
#include <rl_tools/rl/environments/l2f/parameters/dynamics/crazyflie.h>

#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/checkpoint/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

#include <rl_tools/rl/utils/evaluation/operations_cpu.h>


#include <rl_tools/utils/generic/typing.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using TI = typename DEVICE::index_t;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
constexpr TI BASE_SEED = 0;


constexpr bool IDENT = false;
constexpr bool ZERO_ANGLE_INIT = false;
constexpr bool SAMPLE_ENV_PARAMETERS = IDENT || true;
constexpr bool SEQUENTIAL = false;

constexpr static auto MODEL = rl_tools::rl::environments::l2f::parameters::dynamics::REGISTRY::crazyflie;

constexpr static auto MODEL_NAME = rl_tools::rl::environments::l2f::parameters::dynamics::registry_name<MODEL>;
static constexpr auto reward_function = IDENT ? rl_tools::rl::environments::l2f::parameters::reward_functions::squared<T> : rl_tools::rl::environments::l2f::parameters::reward_functions::squared<T>;
using REWARD_FUNCTION_CONST = typename rl_tools::utils::typing::remove_cv_t<decltype(reward_function)>;
using REWARD_FUNCTION = typename rl_tools::utils::typing::remove_cv<REWARD_FUNCTION_CONST>::type;

struct DR_OPTIONS{
    static constexpr bool ENABLED = false;
    static constexpr bool THRUST_TO_WEIGHT = ENABLED;
    static constexpr bool MASS = ENABLED;
    static constexpr bool TORQUE_TO_INERTIA = ENABLED;
    static constexpr bool MASS_SIZE_DEVIATION = ENABLED;
    static constexpr bool ROTOR_TORQUE_CONSTANT = false;
    static constexpr bool DISTURBANCE_FORCE = false;
    static constexpr bool ROTOR_TIME_CONSTANT = false;
};
using PARAMETERS_SPEC = rl_tools::rl::environments::l2f::ParametersBaseSpecification<T, TI, 4, REWARD_FUNCTION>;
using PARAMETERS_TYPE = rl_tools::rl::environments::l2f::ParametersDomainRandomization<rl_tools::rl::environments::l2f::ParametersDomainRandomizationSpecification<T, TI, DR_OPTIONS, rl_tools::rl::environments::l2f::ParametersDisturbances<rlt::rl::environments::l2f::ParametersSpecification<T, TI, rl_tools::rl::environments::l2f::ParametersBase<PARAMETERS_SPEC>>>>>;

static constexpr typename PARAMETERS_TYPE::Dynamics dynamics = rl_tools::rl::environments::l2f::parameters::dynamics::registry<MODEL, PARAMETERS_SPEC>;
static constexpr typename PARAMETERS_TYPE::Integration integration = {
    0.01 // integration dt
};
static constexpr typename PARAMETERS_TYPE::MDP::Initialization init = IDENT ? rl_tools::rl::environments::l2f::parameters::init::init_90_deg<PARAMETERS_SPEC> : (ZERO_ANGLE_INIT ? rl_tools::rl::environments::l2f::parameters::init::init_0_deg<PARAMETERS_SPEC> : rl_tools::rl::environments::l2f::parameters::init::init_90_deg<PARAMETERS_SPEC>);
static constexpr typename PARAMETERS_TYPE::MDP::ObservationNoise observation_noise = {
    0, // 0.01, // position
    0, // 0.001, // orientation
    0, // 0.01, // linear_velocity
    0, // 0.01, // angular_velocity
    0, // 0.0, // imu acceleration
};
//        static constexpr typename PARAMETERS_TYPE::MDP::ObservationNoise observation_noise = {
//                0.0, // position
//                0.0, // orientation
//                0.0, // linear_velocity
//                0.0, // angular_velocity
//                0.0, // imu acceleration
//        };
static constexpr typename PARAMETERS_TYPE::MDP::ActionNoise action_noise = {
    0, // std of additive gaussian noise onto the normalized action (-1, 1)
};
static constexpr typename PARAMETERS_TYPE::MDP::Termination termination = IDENT ? rl_tools::rl::environments::l2f::parameters::termination::fast_learning<PARAMETERS_SPEC> : rl_tools::rl::environments::l2f::parameters::termination::narrow<PARAMETERS_SPEC>;
static constexpr typename PARAMETERS_TYPE::MDP mdp = {
    init,
    reward_function,
    observation_noise,
    action_noise,
    termination
};
static constexpr rlt::rl::environments::l2f::parameters::DomainRandomization<T> domain_randomization = {
    true || IDENT ? 0 : 2.0, // thrust_to_weight_min;
    true || IDENT ? 0 : 5, // thrust_to_weight_max;
    true || IDENT ? 0 : 0.0026034812863058926, // thrust_to_weight_by_torque_to_inertia_min;
    true || IDENT ? 0 : 0.025, // thrust_to_weight_by_torque_to_inertia_max;
    true || IDENT ? 0.0 : 0.02, // mass_min;
    true || IDENT ? 0.0 : 5, // mass_max;
    true || IDENT ? 0.0 : 0.1, // mass_size_deviation;
    0, // motor_time_constant_rising_min;
    0, // motor_time_constant_rising_max;
    0, // motor_time_constant_falling_min;
    0, // motor_time_constant_falling_max;
    0, // rotor_torque_constant_min;
    0, // rotor_torque_constant_max;
    0, // orientation_offset_angle_max;
    0  // disturbance_force_max;
};
static constexpr typename PARAMETERS_TYPE::Disturbances disturbances = {
    typename PARAMETERS_TYPE::Disturbances::UnivariateGaussian{0, 0}, // random_force;
    typename PARAMETERS_TYPE::Disturbances::UnivariateGaussian{0, 0} // random_torque;
};
static constexpr PARAMETERS_TYPE nominal_parameters = {
    {
        {
            dynamics,
            integration,
            mdp,
        },
        disturbances
    },
    domain_randomization
};

namespace static_builder{
    using namespace rl_tools::rl::environments::l2f;
    struct ENVIRONMENT_STATIC_PARAMETERS{
        static constexpr TI N_SUBSTEPS = 1;
        static constexpr TI ACTION_HISTORY_LENGTH = SEQUENTIAL ? 1 : 16;
        static constexpr TI EPISODE_STEP_LIMIT = 500;
        static constexpr TI CLOSED_FORM = false;
        using STATE_BASE = StateBase<StateSpecification<T, TI>>;
        using STATE_TYPE = StateRotorsHistory<StateRotorsHistorySpecification<T, TI, ACTION_HISTORY_LENGTH, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI, STATE_BASE>>>>;
        using OBSERVATION_TYPE = observation::Position<observation::PositionSpecification<T, TI,
                observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecification<T, TI,
                        observation::LinearVelocity<observation::LinearVelocitySpecification<T, TI,
                                observation::AngularVelocity<observation::AngularVelocitySpecification<T, TI,
                                        observation::ActionHistory<observation::ActionHistorySpecification<T, TI, ACTION_HISTORY_LENGTH>>>>>>>>>>;
        using OBSERVATION_TYPE_PRIVILEGED = observation::Position<observation::PositionSpecificationPrivileged<T, TI,
                observation::OrientationRotationMatrix<observation::OrientationRotationMatrixSpecificationPrivileged<T, TI,
                        observation::LinearVelocity<observation::LinearVelocitySpecificationPrivileged<T, TI,
                                observation::AngularVelocity<observation::AngularVelocitySpecificationPrivileged<T, TI,
                                        observation::RandomForce<observation::RandomForceSpecification<T, TI,
                                                observation::RotorSpeeds<observation::RotorSpeedsSpecification<T, TI>>
                                        >
                                        >
                                >>
                        >>
                >>
        >>;
        static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
        using PARAMETERS = PARAMETERS_TYPE;
        static constexpr auto PARAMETER_VALUES = nominal_parameters;
        static constexpr T STATE_LIMIT_POSITION = 100000;
        static constexpr T STATE_LIMIT_VELOCITY = 100000;
        static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
    };
}

using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI, static_builder::ENVIRONMENT_STATIC_PARAMETERS>;
using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;

struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI>{
        static constexpr TI ACTOR_BATCH_SIZE = 64;
        static constexpr TI CRITIC_BATCH_SIZE = 64;
        static constexpr TI TRAINING_INTERVAL = 5;
        static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
        static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
        static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
        static constexpr T TARGET_NEXT_ACTION_NOISE_CLIP = 0.9;
        static constexpr T TARGET_NEXT_ACTION_NOISE_STD = 0.3;
        static constexpr T GAMMA = 0.99;
        static constexpr bool IGNORE_TERMINATION = false;
        static constexpr T TARGET_ENTROPY = -((T)4);
        static constexpr TI SEQUENCE_LENGTH = SEQUENTIAL ? 1 : 1;
    };
    static constexpr TI STEP_LIMIT = 200000;
    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
    static constexpr TI ACTOR_NUM_LAYERS = 4;
    static constexpr TI ACTOR_HIDDEN_DIM = 32;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI CRITIC_NUM_LAYERS = 4;
    static constexpr TI CRITIC_HIDDEN_DIM = 32;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI EPISODE_STEP_LIMIT = 500;
//            static constexpr bool SHARED_BATCH = false;
    struct OPTIMIZER_PARAMETERS_COMMON: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
        static constexpr T GRADIENT_CLIP_VALUE = 1;
        static constexpr bool ENABLE_WEIGHT_DECAY = true;
        static constexpr T WEIGHT_DECAY = 0.0001;
    };
    struct ACTOR_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
        static constexpr T ALPHA = 1e-3;
    };
    struct CRITIC_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
        static constexpr T ALPHA = 1e-3;
    };
    struct ALPHA_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
        static constexpr T ALPHA = 1e-3;
    };
    static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = SAMPLE_ENV_PARAMETERS;
};

using LOOP_CORE_CONFIG_MLP = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP>;
// using LOOP_CORE_CONFIG_GRU = rlt::rl::algorithms::sac::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsGRU>;
// using LOOP_CORE_CONFIG = rlt::utils::typing::conditional_t<SEQUENTIAL, LOOP_CORE_CONFIG_GRU, LOOP_CORE_CONFIG_MLP>;
using LOOP_CORE_CONFIG = LOOP_CORE_CONFIG_MLP;

constexpr TI NUM_CHECKPOINTS = 10;
constexpr TI NUM_EVALUATIONS = 20;
constexpr TI NUM_SAVE_TRAJECTORIES = 20;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG, rlt::rl::loop::steps::timing::Parameters<TI, 10000>>;
using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_TIMING_CONFIG>;
struct LOOP_CHECKPOINT_PARAMETERS: rlt::rl::loop::steps::checkpoint::Parameters<TYPE_POLICY, TI>{
    static constexpr TI CHECKPOINT_INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_CHECKPOINTS;
    static constexpr TI CHECKPOINT_INTERVAL = CHECKPOINT_INTERVAL_TEMP == 0 ? 1 : CHECKPOINT_INTERVAL_TEMP;
};
using LOOP_CHECKPOINT_CONFIG = rlt::rl::loop::steps::checkpoint::Config<LOOP_EXTRACK_CONFIG, LOOP_CHECKPOINT_PARAMETERS>;
struct LOOP_EVALUATION_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CHECKPOINT_CONFIG>{
    static constexpr TI EVALUATION_INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_EVALUATIONS;
    static constexpr TI EVALUATION_INTERVAL = EVALUATION_INTERVAL_TEMP == 0 ? 1 : EVALUATION_INTERVAL_TEMP;
    static constexpr TI NUM_EVALUATION_EPISODES = 100;
    static constexpr TI N_EVALUATIONS = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
    static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = SAMPLE_ENV_PARAMETERS;
};
using LOOP_EVALUATION_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CHECKPOINT_CONFIG, LOOP_EVALUATION_PARAMETERS>;
struct LOOP_SAVE_TRAJECTORIES_PARAMETERS: rlt::rl::loop::steps::save_trajectories::Parameters<TYPE_POLICY, TI, LOOP_CHECKPOINT_CONFIG>{
    static constexpr TI INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / NUM_SAVE_TRAJECTORIES;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
    static constexpr TI NUM_EPISODES = 100;
};
using LOOP_SAVE_TRAJECTORIES_CONFIG = rlt::rl::loop::steps::save_trajectories::Config<LOOP_EVALUATION_CONFIG, LOOP_SAVE_TRAJECTORIES_PARAMETERS>;
#ifdef BENCHMARK
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using LOOP_CONFIG = LOOP_SAVE_TRAJECTORIES_CONFIG;
#endif

using LOOP_STATE = typename LOOP_CONFIG::State<LOOP_CONFIG>;

int main(int argc, char** argv){
    TI arg_seed = 11;
    if(argc > 1){
        arg_seed = std::stoi(argv[1]);
    }
    TI seed = IDENT ? 11 : arg_seed;
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);
    LOOP_STATE ts;
#ifndef BENCHMARK
    ts.extrack_config.name = "dr-sac";
    ts.extrack_config.population_variates = "algorithm_environment_zero-init";
    ts.extrack_config.population_values = "sac_l2f_" + std::string((ZERO_ANGLE_INIT ? "true" : "false"));
#endif
    rlt::malloc(device);
    rlt::init(device);
    rlt::malloc(device, ts);
    rlt::init(device, ts, seed);
#if defined(RL_TOOLS_ENABLE_TENSORBOARD)
    rlt::init(device, device.logger, ts.extrack_paths.seed);
#endif
    std::string parameters_json;
    if constexpr(!SAMPLE_ENV_PARAMETERS) {
        LOOP_CONFIG::ENVIRONMENT env;
        LOOP_CONFIG::ENVIRONMENT::Parameters env_parameters, env_parameters_nominal;
        rlt::initial_parameters(device, env, env_parameters_nominal);
        rlt::sample_initial_parameters(device, env, env_parameters, rng);
        parameters_json = rlt::json(device, env, env_parameters);
        rlt::compare_parameters(device, env_parameters_nominal, env_parameters);
        rlt::set_parameters(device, ts.off_policy_runner, env_parameters);
#ifndef BENCHMARK
        ts.env_eval_parameters = env_parameters;
#endif
    }
    while(!rlt::step(device, ts)){

        bool set_max_angle = false;
        T max_angle = 0;
        // if(ts.step == 1000000){
        //     set_max_angle = true;
        //     max_angle = 22.5/180.0*rlt::math::PI<T>;
        // }
        // if(ts.step == 2000000){
        //     set_max_angle = true;
        //     max_angle = 45.0/180.0*rlt::math::PI<T>;
        // }
        // if(ts.step == 3000000){
        //     set_max_angle = true;
        //     max_angle = 90.0/180.0*rlt::math::PI<T>;
        // }
        if(set_max_angle){
            for(TI env_i=0; env_i < decltype(ts.off_policy_runner)::N_ENVIRONMENTS; env_i++){
                auto& env = get(ts.off_policy_runner.envs, 0, env_i);
                env.parameters.mdp.init.max_angle = max_angle;
            }
#ifndef BENCHMARK
            ts.env_eval.parameters.mdp.init.max_angle = max_angle;
#endif
        }
    }
#ifndef BENCHMARK
    std::filesystem::create_directories(ts.extrack_paths.seed);
    std::ofstream return_file(ts.extrack_paths.seed / "return.json");
    return_file << "{";
    if constexpr(!SAMPLE_ENV_PARAMETERS) {
        return_file << "\"parameters\": " << parameters_json << ", ";
    }
    return_file << "\"evaluation\": ";
    return_file << "[";
    for(TI evaluation_i = 0; evaluation_i < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS; evaluation_i++){
        auto& result = get(ts.evaluation_results, 0, evaluation_i);
        return_file << rlt::json(device, result, LOOP_CONFIG::EVALUATION_PARAMETERS::EVALUATION_INTERVAL * LOOP_CONFIG::ENVIRONMENT_STEPS_PER_LOOP_STEP * evaluation_i);
        if(evaluation_i < LOOP_CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS - 1){
            return_file << ", ";
        }
    }
    return_file << "]";
    return_file << "}";
    std::ofstream return_file_confirmation(ts.extrack_paths.seed / "return.json.set");
    return_file_confirmation.close();
#endif
}