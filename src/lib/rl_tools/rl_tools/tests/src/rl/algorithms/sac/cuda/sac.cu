// /usr/local/cuda/bin/nvcc -I include -DRL_TOOLS_BACKEND_ENABLE_CUDA -lcublas src/rl/environments/pendulum/sac/cuda/sac.cu

#ifdef RL_TOOLS_DEBUG
#define RL_TOOLS_DEBUG_DEVICE_CUDA_SYNCHRONIZE_STATUS_CHECK
#endif
#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_cuda.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/loss_functions/mse/operations_cuda.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_cuda.h>
#include <rl_tools/rl/environments/pendulum/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/nn/optimizers/adam/operations_cuda.h>

#include <rl_tools/rl/components/off_policy_runner/operations_cuda.h>

#include <rl_tools/rl/algorithms/sac/operations_cuda.h>
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

#include <gtest/gtest.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY_CUDA<>;
#ifndef _MSC_VER
using DEVICE_INIT = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE_INIT = rlt::devices::DefaultCPU; // for some reason MKL makes problems in this case (this example seems cursed)
#endif
using TI = typename DEVICE::index_t;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using RNG_INIT = DEVICE_INIT::SPEC::RANDOM::ENGINE<>;

template <typename TYPE_POLICY, typename TI, TI T_STEP_LIMIT>
struct ConfigFactory{
    using T = typename TYPE_POLICY::DEFAULT;
    using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
    using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
    struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
        struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
            static constexpr TI ACTOR_BATCH_SIZE = 100;
            static constexpr TI CRITIC_BATCH_SIZE = 100;
        };
        static constexpr TI STEP_LIMIT = T_STEP_LIMIT;
        static constexpr TI ACTOR_NUM_LAYERS = 3;
        static constexpr TI ACTOR_HIDDEN_DIM = 64;
        static constexpr TI CRITIC_NUM_LAYERS = 3;
        static constexpr TI CRITIC_HIDDEN_DIM = 64;
        static constexpr bool COLLECT_EPISODE_STATS = false;
        static constexpr TI EPISODE_STATS_BUFFER_SIZE = 0;
    };
    template <typename RNG>
    using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;

    struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, LOOP_CORE_CONFIG<RNG>>{
        static constexpr TI NUM_EVALUATION_EPISODES = 100;
    };
    template <typename RNG>
    using LOOP_EVAL_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG<RNG>, LOOP_EVAL_PARAMETERS>;
    template <typename RNG>
    using LOOP_CONFIG = LOOP_EVAL_CONFIG<RNG>;

    using LOOP_STATE = typename LOOP_CONFIG<RNG>::template State<LOOP_CONFIG<RNG>>;
    using LOOP_STATE_INIT = typename LOOP_CONFIG<RNG_INIT>::template State<LOOP_CONFIG<RNG_INIT>>;
};


template <typename TYPE_POLICY, auto STEP_LIMIT, bool GPU_INIT, bool GPU_ROLLOUT, bool GPU_ACTOR_ROLLOUT, bool GPU_TRAINING, bool GPU_NOISE, bool GPU_EVALUATION, bool CPU_TRAINING>
void test(typename TYPE_POLICY::DEFAULT& return_value, typename TYPE_POLICY::DEFAULT epsilon){
    using T = typename TYPE_POLICY::DEFAULT;
    DEVICE device;
    DEVICE_INIT device_init;
    using TI = typename DEVICE::index_t;
    TI seed = 0;
    using CONFIG_FACTORY = ConfigFactory<TYPE_POLICY, TI, STEP_LIMIT>;
    typename CONFIG_FACTORY::LOOP_STATE ts;
    typename CONFIG_FACTORY::LOOP_STATE_INIT ts_init, ts_comparison;
    using CONFIG = typename decltype(ts)::CONFIG;
    using CORE_PARAMETERS = typename CONFIG::CORE_PARAMETERS;
    using EVAL_PARAMETERS = typename CONFIG::EVALUATION_PARAMETERS;
    rlt::init(device);
    rlt::malloc(device, ts);
    rlt::malloc(device_init, ts_init);
    rlt::malloc(device_init, ts_comparison);
    rlt::init(device, ts, 1);
    rlt::check_status(device);
    rlt::init(device_init, ts_init, 1);
    rlt::init(device_init, ts_comparison, 1);
    if constexpr(GPU_INIT) {
        rlt::copy(device, device_init, ts, ts_init);
    }
    else {
        rlt::copy(device_init, device, ts_init, ts);
    }
#ifdef _MSC_VER
    CONFIG::ENVIRONMENT env_eval;
    RNG_INIT rng_eval;
    rlt::rl::environments::DummyUI ui;
#endif
    TI step = 0;
    bool finished = false;
    auto start_time = std::chrono::high_resolution_clock::now();
    return_value = -1337;
    while(!finished){
        // std::cout << "Step: " << step << std::endl;
        // Evaluation
        if(step % 1000 == 0){
            if constexpr(GPU_EVALUATION) {
                rlt::copy(device, device_init, ts.actor_critic.actor, ts_init.actor_critic.actor);
            }
#ifdef _MSC_VER
            using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<T, TI, typename LOOP_STATE::CONFIG::ENVIRONMENT_EVALUATION, EVAL_PARAMETERS::NUM_EVALUATION_EPISODES, CORE_PARAMETERS::EPISODE_STEP_LIMIT>;
            rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
            rlt::evaluate(device_init, env_eval, ui, ts_init.actor_critic.actor, result, ts_init.actor_deterministic_evaluation_buffers, rng_eval, false);
//            auto result = rlt::evaluate(device_init, env_eval, ui, ts_init.actor_critic.actor, rlt::rl::utils::evaluation::Specification<EVAL_PARAMETERS::NUM_EVALUATION_EPISODES, CORE_PARAMETERS::EPISODE_STEP_LIMIT>(), ts_init.actor_deterministic_evaluation_buffers, rng_eval, false);
#else
            using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename decltype(ts)::CONFIG::ENVIRONMENT_EVALUATION, EVAL_PARAMETERS::NUM_EVALUATION_EPISODES, CORE_PARAMETERS::EPISODE_STEP_LIMIT>;
            rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
            rlt::evaluate(device_init, ts_init.env_eval, ts_init.ui, ts_init.actor_critic.actor, result, ts_init.rng_eval, rlt::Mode<rlt::mode::Evaluation<>>{});
#endif
            rlt::log(device_init, device_init.logger, "Step: ", step, " Mean return: ", result.returns_mean);
            return_value = result.returns_mean;
        }

        // Training
        rlt::set_step(device, device.logger, step);
        if constexpr(GPU_ROLLOUT){
            if constexpr(!GPU_ACTOR_ROLLOUT) {
                rlt::copy(device_init, device, ts_init.actor_critic.actor, ts.actor_critic.actor);
            }
            rlt::step<1>(device, ts.off_policy_runner, ts.actor_critic.actor, ts.actor_buffers_eval, ts.rng);
        }
        else {
            if constexpr(GPU_ACTOR_ROLLOUT){
                rlt::copy(device, device_init, ts.actor_critic.actor, ts_init.actor_critic.actor);
            }
            rlt::step<1>(device_init, ts_init.off_policy_runner, ts_init.actor_critic.actor, ts_init.actor_buffers_eval, ts_init.rng);
        }
        if(step > CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS){
            if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::CRITIC_TRAINING_INTERVAL == 0) {
                for(int critic_i = 0; critic_i < 2; critic_i++){
                    if constexpr(GPU_ROLLOUT) {
                        rlt::gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
                        if constexpr(CPU_TRAINING){
                            rlt::copy(device, device_init, ts.critic_batch, ts_init.critic_batch);
                        }
                    }
                    else {
                        rlt::gather_batch(device_init, ts_init.off_policy_runner, ts_init.critic_batch, ts_init.rng);
                        rlt::copy(device_init, device, ts_init.critic_batch, ts.critic_batch);
                    }
                    if constexpr(GPU_NOISE) {
                        rlt::randn(device, ts.action_noise_critic, ts.rng);
                        if constexpr(CPU_TRAINING){
                            rlt::copy(device, device_init, ts.action_noise_critic, ts_init.action_noise_critic);
                        }
                    }
                    else {
                        rlt::randn(device_init, ts_init.action_noise_critic, ts_init.rng);
                        rlt::copy(device_init, device, ts_init.action_noise_critic, ts.action_noise_critic);
                    }
                    if constexpr(GPU_TRAINING) {
                        rlt::train_critic(device, ts.actor_critic, ts.actor_critic.critics[critic_i], ts.critic_batch, ts.actor_critic.critic_optimizers[critic_i], ts.actor_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_training_buffers[critic_i], ts.action_noise_critic, ts.rng);
                    }
                    if constexpr(CPU_TRAINING){
                        rlt::train_critic(device_init, ts_init.actor_critic, ts_init.actor_critic.critics[critic_i], ts_init.critic_batch, ts_init.actor_critic.critic_optimizers[critic_i], ts_init.actor_buffers[critic_i], ts_init.critic_buffers[critic_i], ts_init.critic_buffers[critic_i], ts_init.critic_training_buffers[critic_i], ts_init.action_noise_critic, ts_init.rng);
                    }

                    if(GPU_TRAINING && CPU_TRAINING && (step % (CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::CRITIC_TRAINING_INTERVAL * 100) == 0)) {
                        rlt::copy(device, device_init, ts, ts_comparison);
                        T abs_diff = rlt::abs_diff(device_init, ts_init.actor_critic.critics[0], ts_comparison.actor_critic.critics[0]);
                        std::cout << "Abs diff is: " << abs_diff << " after critic update" << std::endl;
                        ASSERT_LT(abs_diff, epsilon);
                    }
                }
            }
            if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::ACTOR_TRAINING_INTERVAL == 0) {
                {
                    if constexpr(GPU_ROLLOUT) {
                        rlt::gather_batch(device, ts.off_policy_runner, ts.actor_batch, ts.rng);
                        if constexpr(CPU_TRAINING){
                            rlt::copy(device, device_init, ts.actor_batch, ts_init.actor_batch);
                        }
                    }
                    else {
                        rlt::gather_batch(device_init, ts_init.off_policy_runner, ts_init.actor_batch, ts_init.rng);
                        rlt::copy(device_init, device, ts_init.actor_batch, ts.actor_batch);
                    }
                    if constexpr(GPU_NOISE) {
                        rlt::randn(device, ts.action_noise_actor, ts.rng);
                        if constexpr(CPU_TRAINING){
                            rlt::copy(device, device_init, ts.action_noise_actor, ts_init.action_noise_actor);
                        }
                    }
                    else {
                        rlt::randn(device_init, ts_init.action_noise_actor, ts_init.rng);
                        rlt::copy(device_init, device, ts_init.action_noise_actor, ts.action_noise_actor);
                    }
                    if constexpr(GPU_TRAINING) {
                        rlt::train_actor(device, ts.actor_critic, ts.actor_batch, ts.actor_critic.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.action_noise_actor, ts.rng);
                    }
                    if constexpr(CPU_TRAINING){
                        rlt::train_actor(device_init, ts_init.actor_critic, ts_init.actor_batch, ts_init.actor_critic.actor_optimizer, ts_init.actor_buffers[0], ts_init.critic_buffers[0], ts_init.actor_training_buffers, ts_init.action_noise_actor, ts_init.rng);
                    }

                    if(GPU_TRAINING && CPU_TRAINING && (step % (CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::ACTOR_TRAINING_INTERVAL * 100) == 0)){
                        rlt::copy(device, device_init, ts, ts_comparison);
                        T abs_diff = rlt::abs_diff(device_init, ts_init.actor_critic.actor, ts_comparison.actor_critic.actor);
                        std::cout << "Abs diff is: " << abs_diff << " after actor update" << std::endl;
                        ASSERT_LT(abs_diff, epsilon);
                    }
                }
                if constexpr(GPU_TRAINING) {
                    rlt::update_critic_targets(device, ts.actor_critic);
                }
                if constexpr(CPU_TRAINING){
                    rlt::update_critic_targets(device_init, ts_init.actor_critic);
                }
            }
        }
        step++;
        finished = step > CORE_PARAMETERS::STEP_LIMIT;
     }
    rlt::malloc(device, ts);
    rlt::malloc(device_init, ts_init);
    rlt::malloc(device_init, ts_comparison);
}
TEST(RL_TOOLS_RL_ALGORITHMS_SAC_CUDA, CPU_TRAINING) {
    constexpr bool GPU_INIT = false;
    constexpr bool GPU_ROLLOUT = false;
    constexpr bool GPU_ACTOR_ROLLOUT = false;
    constexpr bool GPU_TRAINING = false;
    constexpr bool GPU_NOISE = false;
    constexpr bool GPU_EVALUATION = false;
    constexpr bool CPU_TRAINING = true;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    constexpr T epsilon = 1e-8;
    T return_value;
    test<TYPE_POLICY, 15000, GPU_INIT, GPU_ROLLOUT, GPU_ACTOR_ROLLOUT, GPU_TRAINING, GPU_NOISE, GPU_EVALUATION, CPU_TRAINING>(return_value, epsilon);
    ASSERT_GE(return_value, -200);
}

TEST(RL_TOOLS_RL_ALGORITHMS_SAC_CUDA, GPU_ROLLOUT) {
    constexpr bool GPU_INIT = false;
    constexpr bool GPU_ROLLOUT = true;
    constexpr bool GPU_ACTOR_ROLLOUT = false;
    constexpr bool GPU_TRAINING = false;
    constexpr bool GPU_NOISE = false;
    constexpr bool GPU_EVALUATION = false;
    constexpr bool CPU_TRAINING = true;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    constexpr T epsilon = 1e-10;
    T return_value;
    test<TYPE_POLICY, 15000, GPU_INIT, GPU_ROLLOUT, GPU_ACTOR_ROLLOUT, GPU_TRAINING, GPU_NOISE, GPU_EVALUATION, CPU_TRAINING>(return_value, epsilon);
    ASSERT_GE(return_value, -200);
}

TEST(RL_TOOLS_RL_ALGORITHMS_SAC_CUDA, GPU_INIT_GPU_ACTOR_ROLLOUT_GPU_EVALUATION){
    constexpr bool GPU_INIT = true;
    constexpr bool GPU_ROLLOUT = false;
    constexpr bool GPU_ACTOR_ROLLOUT = true;
    constexpr bool GPU_TRAINING = true;
    constexpr bool GPU_NOISE = false;
    constexpr bool GPU_EVALUATION = true;
    constexpr bool CPU_TRAINING = true;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    constexpr T epsilon = 1e-8;
    T return_value;
    test<TYPE_POLICY, 15000, GPU_INIT, GPU_ROLLOUT, GPU_ACTOR_ROLLOUT, GPU_TRAINING, GPU_NOISE, GPU_EVALUATION, CPU_TRAINING>(return_value, epsilon);
    ASSERT_GE(return_value, -200);
}

TEST(RL_TOOLS_RL_ALGORITHMS_SAC_CUDA, FULL_GPU_TRAINING) {
    constexpr bool GPU_INIT = true;
    constexpr bool GPU_ROLLOUT = true;
    constexpr bool GPU_ACTOR_ROLLOUT = true;
    constexpr bool GPU_TRAINING = true;
    constexpr bool GPU_NOISE = true;
    constexpr bool GPU_EVALUATION = true;
    constexpr bool CPU_TRAINING = false;
    using T = double;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    constexpr T epsilon = 1e10;
    T return_value;
    test<TYPE_POLICY, 15000, GPU_INIT, GPU_ROLLOUT, GPU_ACTOR_ROLLOUT, GPU_TRAINING, GPU_NOISE, GPU_EVALUATION, CPU_TRAINING>(return_value, epsilon);
    ASSERT_GE(return_value, -200);
}

// benchmark training should take < 2s on P1
