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

#include "cuda_graph_export.h"

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY_CUDA<>;
#ifndef _MSC_VER
using DEVICE_EVALUATION = rlt::devices::DEVICE_FACTORY<>;
#else
using DEVICE_INIT = rlt::devices::DefaultCPU; // for some reason MKL makes problems in this case (this example seems cursed)
#endif
DEVICE dummy_device; // this is needed because default_engine can not take a const device
using TI = typename DEVICE::index_t;
// using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<rlt::devices::random::CUDA::Specification<TI, 1024>>;


using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<float, TI, rlt::rl::environments::pendulum::DefaultParameters<float>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT::ACTION_DIM>{
        static constexpr TI ACTOR_BATCH_SIZE = 100;
        static constexpr TI CRITIC_BATCH_SIZE = 100;
    };
    static constexpr TI STEP_LIMIT = 10000;
    static constexpr TI REPLAY_BUFFER_CAP = STEP_LIMIT;
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


int main() {
    TI seed = 0;
    DEVICE device;
    DEVICE_EVALUATION device_evaluation;
    LOOP_STATE ts;
    using CONFIG = typename decltype(ts)::CONFIG;
    using CORE_PARAMETERS = typename CONFIG::CORE_PARAMETERS;
    using EVAL_PARAMETERS = typename CONFIG::EVALUATION_PARAMETERS;
    DEVICE_EVALUATION::SPEC::RANDOM::ENGINE rng_evaluation;
    rlt::init(device_evaluation, rng_evaluation, seed);
    using ACTOR_TYPE_ORIG = rlt::utils::typing::remove_reference_t<decltype(rlt::get_actor(ts))>;
    using ACTOR_TYPE_INFERENCE = ACTOR_TYPE_ORIG:: template CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    using ACTOR_TYPE_EVALUATION = ACTOR_TYPE_INFERENCE:: template CHANGE_BATCH_SIZE<TI, EVAL_PARAMETERS::NUM_EVALUATION_EPISODES>;
    ACTOR_TYPE_EVALUATION actor_evaluation;
    ACTOR_TYPE_EVALUATION::Buffer<> actor_buffers_evaluation;
    ENVIRONMENT env_evaluation;
    ENVIRONMENT::Parameters env_evaluation_parameters;
    rlt::rl::environments::DummyUI ui;
    rlt::init(device);
    rlt::malloc(device, ts);
    rlt::malloc(device_evaluation, actor_evaluation);
    rlt::malloc(device_evaluation, actor_buffers_evaluation);
    rlt::init(device, ts, 1);
    TI step = 0;
    bool finished = false;

    // {
    //     cudaGraph_t test_graph;
    //     cudaStreamBeginCapture(device.stream, cudaStreamCaptureModeGlobal);
    //     device.graph_capture_active = true;
    //     rlt::zero_gradient(device, ts.actor_critic.actor.content.input_layer);
    //     cudaStreamEndCapture(device.stream, &test_graph);
    //     device.graph_capture_active = false;
    //
    //     rlt::check_status(device);
    //     cudaGraphExec_t graphExec;
    //     cudaGraphInstantiate(&graphExec, test_graph, nullptr, nullptr, 0);
    //     cudaGraphLaunch(graphExec, device.stream);
    //     cudaDeviceSynchronize();
    // }

    constexpr bool CUDA_GRAPH = true;
    // constexpr bool CUDA_GRAPH = false;



    if constexpr(CUDA_GRAPH){
        cudaGraph_t step_graph;
        cudaGraphExec_t step_graph_exec;
        {
            cudaStreamBeginCapture(device.stream, cudaStreamCaptureModeGlobal);
            device.graph_capture_active = true;
            rlt::step<1>(device, ts.off_policy_runner, ts.actor_critic.actor, ts.actor_buffers_eval, ts.rng);
            cudaStreamEndCapture(device.stream, &step_graph);
            device.graph_capture_active = false;
            rlt::check_status(device);
            cudaGraphInstantiate(&step_graph_exec, step_graph, nullptr, nullptr, 0);
        }
        cudaGraph_t critic_training_graph;
        cudaGraphExec_t critic_training_graph_exec;
        {
            cudaStreamBeginCapture(device.stream, cudaStreamCaptureModeGlobal);
            device.graph_capture_active = true;
            for(int critic_i = 0; critic_i < 2; critic_i++){
                rlt::gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
                rlt::randn(device, ts.action_noise_critic, ts.rng);
                rlt::train_critic(device, ts.actor_critic, ts.actor_critic.critics[critic_i], ts.critic_batch, ts.actor_critic.critic_optimizers[critic_i], ts.actor_target_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_target_buffers[critic_i], ts.critic_training_buffers[critic_i], ts.action_noise_critic, ts.rng);
            }
            cudaStreamEndCapture(device.stream, &critic_training_graph);
            device.graph_capture_active = false;
            rlt::print_graph(critic_training_graph);
            rlt::check_status(device);
            cudaGraphInstantiate(&critic_training_graph_exec, critic_training_graph, nullptr, nullptr, 0);
        }
        cudaGraph_t actor_training_graph;
        cudaGraphExec_t actor_training_graph_exec;
        {
            cudaStreamBeginCapture(device.stream, cudaStreamCaptureModeGlobal);
            device.graph_capture_active = true;
            rlt::gather_batch(device, ts.off_policy_runner, ts.actor_batch, ts.rng);
            rlt::randn(device, ts.action_noise_actor, ts.rng);
            rlt::train_actor(device, ts.actor_critic, ts.actor_batch, ts.actor_critic.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.action_noise_actor, ts.rng);
            rlt::update_critic_targets(device, ts.actor_critic);
            cudaStreamEndCapture(device.stream, &actor_training_graph);
            device.graph_capture_active = false;
            rlt::check_status(device);
            cudaGraphInstantiate(&actor_training_graph_exec, actor_training_graph, nullptr, nullptr, 0);
            rlt::print_graph(actor_training_graph);
        }
        dumpCudaGraphDOT(step_graph,            "cuda_graph_step.dot");
        dumpCudaGraphDOT(critic_training_graph, "cuda_graph_critic_training.dot");
        dumpCudaGraphDOT(actor_training_graph,  "cuda_traph_actor_training.dot");
        while(!finished){
            // Evaluation
            if(step % 1000 == 0){
                rlt::copy(device, device_evaluation, rlt::get_actor(ts), actor_evaluation);
                cudaStreamSynchronize(device.stream);
                using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename decltype(ts)::CONFIG::ENVIRONMENT_EVALUATION, EVAL_PARAMETERS::NUM_EVALUATION_EPISODES, CORE_PARAMETERS::EPISODE_STEP_LIMIT>;
                rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
                rlt::evaluate(device_evaluation, env_evaluation, ui, actor_evaluation, result, rng_evaluation, rlt::Mode<rlt::mode::Evaluation<>>{});
                rlt::log(device_evaluation, device_evaluation.logger, "Step: ", step, " Mean return: ", result.returns_mean);
            }

            rlt::set_step(device, device.logger, step);
            cudaGraphLaunch(step_graph_exec, device.stream);
            rlt::check_status(device);
            if(step > CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS){
                if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::CRITIC_TRAINING_INTERVAL == 0) {
                    cudaGraphLaunch(critic_training_graph_exec, device.stream);
                    rlt::check_status(device);
                }
                if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::ACTOR_TRAINING_INTERVAL == 0) {
                    cudaGraphLaunch(actor_training_graph_exec, device.stream);
                    rlt::check_status(device);
                }
            }
            step++;
            finished = step > CORE_PARAMETERS::STEP_LIMIT;
         }
    }
    else {
        while(!finished){
            // Evaluation
            if(step % 1000 == 0){
                rlt::copy(device, device_evaluation, rlt::get_actor(ts), actor_evaluation);
                cudaStreamSynchronize(device.stream);
                using RESULT_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, typename decltype(ts)::CONFIG::ENVIRONMENT_EVALUATION, EVAL_PARAMETERS::NUM_EVALUATION_EPISODES, CORE_PARAMETERS::EPISODE_STEP_LIMIT>;
                rlt::rl::utils::evaluation::Result<RESULT_SPEC> result;
                rlt::evaluate(device_evaluation, env_evaluation, ui, actor_evaluation, result, rng_evaluation, rlt::Mode<rlt::mode::Evaluation<>>{});
                rlt::log(device_evaluation, device_evaluation.logger, "Step: ", step, " Mean return: ", result.returns_mean);
            }

            // Training
            rlt::set_step(device, device.logger, step);
            rlt::step<1>(device, ts.off_policy_runner, ts.actor_critic.actor, ts.actor_buffers_eval, ts.rng);
            if(step > CONFIG::CORE_PARAMETERS::N_WARMUP_STEPS){
                if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::CRITIC_TRAINING_INTERVAL == 0) {
                    for(TI critic_i = 0; critic_i < 2; critic_i++){
                        rlt::gather_batch(device, ts.off_policy_runner, ts.critic_batch, ts.rng);
                        rlt::randn(device, ts.action_noise_critic, ts.rng);
                        rlt::train_critic(device, ts.actor_critic, ts.actor_critic.critics[critic_i], ts.critic_batch, ts.actor_critic.critic_optimizers[critic_i], ts.actor_target_buffers[critic_i], ts.critic_buffers[critic_i], ts.critic_target_buffers[critic_i], ts.critic_training_buffers[critic_i], ts.action_noise_critic, ts.rng);
                    }
                }
                if(step % CONFIG::CORE_PARAMETERS::SAC_PARAMETERS::ACTOR_TRAINING_INTERVAL == 0) {
                    rlt::gather_batch(device, ts.off_policy_runner, ts.actor_batch, ts.rng);
                    rlt::randn(device, ts.action_noise_actor, ts.rng);
                    rlt::train_actor(device, ts.actor_critic, ts.actor_batch, ts.actor_critic.actor_optimizer, ts.actor_buffers[0], ts.critic_buffers[0], ts.actor_training_buffers, ts.action_noise_actor, ts.rng);
                    rlt::update_critic_targets(device, ts.actor_critic);
                }
            }
            step++;
            finished = step > CORE_PARAMETERS::STEP_LIMIT;
         }

    }
    rlt::free(device, ts);
    return 0;
}

// benchmark training should take < 2s on P1
