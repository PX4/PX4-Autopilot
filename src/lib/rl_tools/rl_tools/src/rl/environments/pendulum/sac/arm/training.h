#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
#define RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
#endif

#ifdef RL_TOOLS_DEPLOYMENT_ARDUINO
#include <rl_tools/operations/arm.h>
#include <rl_tools/logging/operations_arduino.h>
#else
#define RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION
#include <rl_tools/version.h>
#include <rl_tools/rl_tools.h>
#include <rl_tools/operations/arm/group_1.h>
#include <rl_tools/operations/cpu/group_1.h>
#include <rl_tools/operations/arm/group_2.h>
#include <rl_tools/operations/cpu/group_2.h>
#include <rl_tools/operations/arm/group_3.h>
#include <rl_tools/operations/cpu/group_3.h>
#endif

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>


#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

#ifdef RL_TOOLS_DEPLOYMENT_ARDUINO
using LOGGING = rlt::devices::logging::ARDUINO;
using DEVICE = rlt::devices::arm::OPT<rlt::devices::arm::Specification<rlt::devices::math::ARM, rlt::devices::random::ARM, LOGGING>>;
#else
using DEVICE = rlt::devices::DefaultCPU;
#endif

using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
constexpr bool DYNAMIC_ALLOCATION_ACTOR = false;
constexpr bool DYNAMIC_ALLOCATION_CRITIC = false;
constexpr bool DYNAMIC_ALLOCATION_LOOP_STATE = false;
#endif

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr TI ACTOR_BATCH_SIZE = 100;
        static constexpr TI CRITIC_BATCH_SIZE = 100;
    };
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI ACTOR_NUM_LAYERS = 3;
    static constexpr TI ACTOR_HIDDEN_DIM = 64;
    static constexpr TI CRITIC_NUM_LAYERS = 3;
    static constexpr TI CRITIC_HIDDEN_DIM = 64;
};
template<typename T, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION>
struct APPROXIMATOR_CONFIG{
    template <typename CAPABILITY>
    struct Actor{
        using ACTOR_INPUT_SHAPE = rlt::tensor::Shape<TI, 1, PARAMETERS::SAC_PARAMETERS::ACTOR_BATCH_SIZE, ENVIRONMENT::Observation::DIM>;
        using ACTOR_SPEC = rlt::nn_models::mlp::Configuration<T, TI, 2*ENVIRONMENT::ACTION_DIM, PARAMETERS::ACTOR_NUM_LAYERS, PARAMETERS::ACTOR_HIDDEN_DIM, PARAMETERS::ACTOR_ACTIVATION_FUNCTION,  rlt::nn::activation_functions::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<T, TI>>;
        using ACTOR_TYPE = rlt::nn_models::mlp::BindConfiguration<ACTOR_SPEC>;
        using SAMPLE_AND_SQUASH_LAYER_SPEC = rlt::nn::layers::sample_and_squash::Configuration<T, TI, rlt::nn::layers::sample_and_squash::DefaultParameters<T>>;
        using SAMPLE_AND_SQUASH_LAYER = rlt::nn::layers::sample_and_squash::BindConfiguration<SAMPLE_AND_SQUASH_LAYER_SPEC>;

        template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
        using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
        using MODULE_CHAIN = Module<ACTOR_TYPE, Module<SAMPLE_AND_SQUASH_LAYER>>;

        using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, ACTOR_INPUT_SHAPE>;
    };
    template <typename CAPABILITY>
    struct Critic{
        static constexpr TI INPUT_DIM = ENVIRONMENT::Observation::DIM+ENVIRONMENT::ACTION_DIM;
        using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, PARAMETERS::SAC_PARAMETERS::CRITIC_BATCH_SIZE, INPUT_DIM>;
        using SPEC = rlt::nn_models::mlp::Configuration<T, TI, 1, PARAMETERS::CRITIC_NUM_LAYERS, PARAMETERS::CRITIC_HIDDEN_DIM, PARAMETERS::CRITIC_ACTIVATION_FUNCTION, rlt::nn::activation_functions::IDENTITY, rlt::nn::layers::dense::DefaultInitializer<T, TI>>;
        using TYPE = rlt::nn_models::mlp::BindConfiguration<SPEC>;
        template <typename T_CONTENT, typename T_NEXT_MODULE = rlt::nn_models::sequential::OutputModule>
        using Module = typename rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;
        using MODULE_CHAIN = Module<TYPE>;
        using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
    };

    using CAPABILITY_ACTOR = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, DYNAMIC_ALLOCATION_ACTOR>;
    using CAPABILITY_CRITIC = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, DYNAMIC_ALLOCATION_CRITIC>;
    using ACTOR_TYPE = typename Actor<CAPABILITY_ACTOR>::MODEL;
    using CRITIC_TYPE = typename Critic<CAPABILITY_CRITIC>::MODEL;
    using CRITIC_TARGET_TYPE = typename Critic<rlt::nn::capability::Forward<DYNAMIC_ALLOCATION_CRITIC>>::MODEL;
    using ACTOR_OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::ACTOR_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION_LOOP_STATE>>;
    using CRITIC_OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::CRITIC_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION_LOOP_STATE>>;
    using ALPHA_OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<T, TI, typename PARAMETERS::ALPHA_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION_LOOP_STATE>>;

};

using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, APPROXIMATOR_CONFIG, DYNAMIC_ALLOCATION_LOOP_STATE>;
#ifdef BENCHMARK
#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using LOOP_CONFIG = LOOP_CORE_CONFIG;
#endif
#else
template <typename NEXT>
struct EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, NEXT>{
    static constexpr TI EVALUATION_INTERVAL = 1000;
    static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
};
#ifdef BENCHMARK
#endif
using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG, EVAL_PARAMETERS<LOOP_CORE_CONFIG>>;
#ifndef RL_TOOLS_DEPLOYMENT_ARDUINO
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;
#else
using LOOP_CONFIG = LOOP_EVAL_CONFIG;
#endif
#endif

using LOOP_STATE = LOOP_CONFIG::State<LOOP_CONFIG>;

template <typename DEVICE, typename LOOP_STATE>
void print_sizes(DEVICE& device, LOOP_STATE& ts){
    rlt::log(device, device.logger, "ActorCritic size: ", sizeof(ts.actor_critic));
    rlt::log(device, device.logger, "ActorCritic.actor size: ", sizeof(ts.actor_critic.actor));
    rlt::log(device, device.logger, "ActorCritic.critics[0] size: ", sizeof(ts.actor_critic.critics[0]));
    rlt::log(device, device.logger, "ActorCritic.critics[1] size: ", sizeof(ts.actor_critic.critics[1]));
    rlt::log(device, device.logger, "ActorCritic.critics_target[0] size: ", sizeof(ts.actor_critic.critics_target[0]));
    rlt::log(device, device.logger, "ActorCritic.critics_target[1] size: ", sizeof(ts.actor_critic.critics_target[1]));
    rlt::log(device, device.logger, "OffPolicyRunner size: ", sizeof(ts.off_policy_runner));
    rlt::log(device, device.logger, "OffPolicyRunner.replay_buffers size: ", sizeof(ts.off_policy_runner.replay_buffers));
    rlt::log(device, device.logger, "CriticBatch size: ", sizeof(ts.critic_batch));
    rlt::log(device, device.logger, "CriticTrainingBuffers size: ", sizeof(ts.critic_training_buffers));
    rlt::log(device, device.logger, "CriticBuffers size: ", sizeof(ts.critic_buffers));
    rlt::log(device, device.logger, "ActorBatch size: ", sizeof(ts.actor_batch));
    rlt::log(device, device.logger, "ActorTrainingBuffers size: ", sizeof(ts.actor_training_buffers));
    rlt::log(device, device.logger, "ActorBuffers size: ", sizeof(ts.actor_buffers));
    rlt::log(device, device.logger, "ActorBuffersEval size: ", sizeof(ts.actor_buffers_eval));
    TI accounted_for = sizeof(ts.actor_critic) + sizeof(ts.off_policy_runner) + sizeof(ts.critic_batch) + sizeof(ts.critic_training_buffers) + sizeof(ts.critic_buffers) + sizeof(ts.actor_batch) + sizeof(ts.actor_training_buffers) + sizeof(ts.actor_buffers) + sizeof(ts.actor_buffers_eval);
    rlt::log(device, device.logger, "Total (accounted for): ", accounted_for);
    rlt::log(device, device.logger, "Total: ", sizeof(ts));
}

void train(){
    DEVICE device;
    LOOP_STATE ts;
    rlt::malloc(device, ts);
    rlt::init(device, ts, 0);
    print_sizes(device, ts);
    while(!rlt::step(device, ts)){
    }
    rlt::free(device, ts);
}


// benchmark training should take < 2s on P1, < 0.75 on M3
