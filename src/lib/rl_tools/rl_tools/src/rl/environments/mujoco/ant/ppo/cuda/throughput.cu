#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn_models/operations_cpu.h>
#include <rl_tools/nn_models/persist.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;
#include "../parameters_ppo.h"
#ifdef RL_TOOLS_BACKEND_ENABLE_MKL
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_mkl.h>
#else
#ifdef RL_TOOLS_BACKEND_ENABLE_ACCELERATE
#include <rl_tools/rl/components/on_policy_runner/operations_cpu_accelerate.h>
#else
#include <rl_tools/rl/components/on_policy_runner/operations_cpu.h>
#endif
#endif
#include <rl_tools/rl/algorithms/ppo/operations_generic.h>
#include <rl_tools/rl/utils/evaluation/operations_generic.h>

#include <gtest/gtest.h>
#include <highfive/H5File.hpp>


namespace parameters = parameters_0;

using LOGGER = rlt::devices::logging::CPU_TENSORBOARD<>;

using DEV_SPEC_SUPER = rlt::devices::cpu::Specification<rlt::devices::math::CPU, rlt::devices::random::CPU, LOGGER>;
using TI = typename rlt::devices::DEVICE_FACTORY<DEV_SPEC_SUPER>::index_t;
namespace execution_hints{
    struct HINTS: rlt::rl::components::on_policy_runner::ExecutionHints<TI, 16>{};
}
struct DEV_SPEC: DEV_SPEC_SUPER{
    using EXECUTION_HINTS = execution_hints::HINTS;
};
using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;


using DEVICE = rlt::devices::DEVICE_FACTORY<DEV_SPEC>;
using DEVICE_CUDA = rlt::devices::DEVICE_FACTORY_GPU<rlt::devices::DefaultCUDASpecification>;
using T = float;
using TI = typename DEVICE::index_t;
using envp = parameters::environment<double, TI>;
using rlp = parameters::rl<T, TI, envp::ENVIRONMENT>;
using STATE = envp::ENVIRONMENT::State;

TEST(RL_TOOLS_RL_ENVIRONMENTS_MUJOCO_ANT, THROUGHPUT_MULTI_CORE_SPAWNING_CUDA){
    constexpr TI NUM_ROLLOUT_STEPS = 760;
    constexpr TI NUM_STEPS_PER_ENVIRONMENT = 64;
    constexpr TI NUM_ENVIRONMENTS = 64;
    constexpr TI NUM_THREADS = 16;
    using ACTOR_STRUCTURE_SPEC = rlt::nn_models::mlp::StructureSpecification<T, TI, envp::ENVIRONMENT::Observation::DIM, envp::ENVIRONMENT::ACTION_DIM, 3, 256, rlt::nn::activation_functions::ActivationFunction::RELU, rlt::nn::activation_functions::IDENTITY>;
    using ACTOR_SPEC = rlt::nn_models::mlp::AdamSpecification<ACTOR_STRUCTURE_SPEC>;
    using ACTOR_TYPE = rlt::nn_models::mlp_unconditional_stddev::NeuralNetworkAdam<ACTOR_SPEC>;

    DEVICE device;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    STATE states[NUM_ENVIRONMENTS], next_states[NUM_ENVIRONMENTS];
    envp::ENVIRONMENT envs[NUM_ENVIRONMENTS];
    ACTOR_TYPE actor_cpu, actor_gpu;
    ACTOR_TYPE::Buffers<NUM_ENVIRONMENTS> actor_buffers;
    rlt::Matrix<rlt::matrix::Specification<T, TI, NUM_ENVIRONMENTS, envp::ENVIRONMENT::ACTION_DIM>> actions, actions_gpu;
    rlt::Matrix<rlt::matrix::Specification<T, TI, NUM_ENVIRONMENTS, envp::ENVIRONMENT::Observation::DIM>> observations, observations_gpu;
    auto proto_rng = rlt::random::default_engine(DEVICE{}, 10);
    decltype(proto_rng) rngs[NUM_THREADS];

    rlt::malloc(device, actions);
    rlt::malloc(device, observations);
    rlt::malloc(device, actor_cpu);
    rlt::malloc(device_cuda, actions_gpu);
    rlt::malloc(device_cuda, observations_gpu);
    rlt::malloc(device_cuda, actor_gpu);
    rlt::malloc(device_cuda, actor_buffers);
    for(TI env_i = 0; env_i < NUM_ENVIRONMENTS; env_i++){
        rlt::malloc(device, envs[env_i]);
    }

    rlt::randn(device, actions, proto_rng);
    rlt::init_weights(device, actor_cpu, proto_rng);
    rlt::copy(device, device_cuda, actor_cpu, actor_gpu);

    for(TI env_i = 0; env_i < NUM_ENVIRONMENTS; env_i++){
        rlt::sample_initial_state(device, envs[env_i], states[env_i], proto_rng);
        auto observation = rlt::view(device, observations, rlt::matrix::ViewSpec<1, envp::ENVIRONMENT::Observation::DIM>(), env_i, 0);
        rlt::observe(device,envs[env_i], states[env_i], observation);
    }


    auto start = std::chrono::high_resolution_clock::now();
    for(TI rollout_step_i = 0; rollout_step_i < NUM_ROLLOUT_STEPS; rollout_step_i++){
        rlt::copy(device, device_cuda, actor_cpu, actor_gpu);
        std::cout << "Rollout step " << rollout_step_i << std::endl;
        for(TI step_i = 0; step_i < NUM_STEPS_PER_ENVIRONMENT; step_i++) {
            std::vector<std::thread> threads;
            for(TI thread_i = 0; thread_i < NUM_THREADS; thread_i++){
                threads.emplace_back([&device, &rngs, &actions, &observations, &envs, &states, &next_states, thread_i, step_i](){
                    for(TI env_i = thread_i; env_i < NUM_ENVIRONMENTS; env_i += NUM_THREADS){
                        auto rng = rngs[thread_i];
                        auto& env = envs[env_i];
                        auto& state = states[env_i];
                        auto& next_state = next_states[env_i];
                        auto action = rlt::view(device, actions, rlt::matrix::ViewSpec<1, envp::ENVIRONMENT::ACTION_DIM>(), env_i, 0);
                        auto observation = rlt::view(device, observations, rlt::matrix::ViewSpec<1, envp::ENVIRONMENT::Observation::DIM>(), env_i, 0);
                        rlt::step(device, env, state, action, next_state);
                        if(step_i % 1000 == 0 || rlt::terminated(device, env, next_state, rng)) {
                            rlt::sample_initial_state(device, env, state, rng);
                        }
                        else{
                            next_state = state;
                        }
                        rlt::observe(device, env, next_state, observation);
                    }
                });
            }
            for(TI env_i = 0; env_i < NUM_THREADS; env_i++){
                threads[env_i].join();
            }
            rlt::copy(device, device_cuda, observations, observations_gpu);
            rlt::evaluate(device_cuda, actor_gpu, observations_gpu, actions_gpu, actor_buffers);
            rlt::copy(device_cuda, device, actions_gpu, actions);
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    auto steps_per_second = NUM_STEPS_PER_ENVIRONMENT * NUM_ENVIRONMENTS * NUM_ROLLOUT_STEPS * 1000.0 / duration.count();
    auto frames_per_second = steps_per_second * envp::ENVIRONMENT::SPEC::PARAMETERS::FRAME_SKIP;
    std::cout << "Throughput: " << steps_per_second << " steps/s (frameskip: " << envp::ENVIRONMENT::SPEC::PARAMETERS::FRAME_SKIP << " -> " << frames_per_second << " fps)" << std::endl;
}
