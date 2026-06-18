#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_COMPONENTS_OFF_POLICY_RUNNER_OPERATIONS_CUDA_H

#include "../../../devices/dummy.h"
#include "operations_generic.h"
#include "off_policy_runner.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::components::off_policy_runner{
        template <typename DEVICE, typename SPEC>
        __global__
        void init_kernel(DEVICE device, rl::components::OffPolicyRunner<SPEC> runner){
            static_assert(decltype(runner.replay_buffers)::T::SPEC::DYNAMIC_ALLOCATION == false, "The replay buffers should be statically allocated when using the OffPolicyRunner using CUDA");
            static_assert(decltype(runner.episode_stats)::T::SPEC::DYNAMIC_ALLOCATION == false, "The episode stats should be statically allocated when using the OffPolicyRunner using CUDA");
            using TI = typename SPEC::TI;
            // if the episode is done (step limit activated for STEP_LIMIT > 0) or if the step is the first step for this runner, reset the environment
            TI env_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(env_i < SPEC::PARAMETERS::N_ENVIRONMENTS){
                auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
                replay_buffer = {}; // zero-initialize such that in debug mode the nullptr check on malloc does not trigger
                init(device, replay_buffer);
                auto& episode_stats = get(runner.episode_stats, 0, env_i);
                episode_stats = {};
                init_views(device, episode_stats);
                init(device, episode_stats);
                auto& env = get(runner.envs, 0, env_i);
                env = {};
                init(device, env);
                auto& parameters = get(runner.env_parameters, 0, env_i);
                initial_parameters(device, env, parameters);
            }
        }
        template <bool DETERMINISTIC, typename DEVICE, typename RUNNER_SPEC, typename BATCH_SPEC, typename RNG>
        __global__
        void gather_batch_kernel(DEVICE device, rl::components::OffPolicyRunner<RUNNER_SPEC> runner, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC> batch, RNG rng) {
            int dummy;
            void* stack_ptr = &dummy;
            // printf("Stack pointer in thread (%d, %d): %p\n", blockIdx.x, threadIdx.x, stack_ptr);
            using TI = typename RUNNER_SPEC::TI;
            using RUNNER = rl::components::OffPolicyRunner<RUNNER_SPEC>;
            // if the episode is done (step limit activated for STEP_LIMIT > 0) or if the step is the first step for this runner, reset the environment
            TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
            static_assert(RNG::NUM_RNGS >= BATCH_SPEC::BATCH_SIZE, "Please increase the number of CUDA RNGs");
            if(batch_step_i < BATCH_SPEC::BATCH_SIZE){
                auto& rng_state = get(rng.states, 0, batch_step_i);
                typename DEVICE::index_t env_i = DETERMINISTIC ? 0 : random::uniform_int_distribution(typename DEVICE::SPEC::RANDOM(), (TI) 0, RUNNER_SPEC::PARAMETERS::N_ENVIRONMENTS - 1, rng_state);
                // printf("Chose env %d\n", env_i);
                auto& replay_buffer = get(runner.replay_buffers, 0, env_i);
                // printf("replay buffer pointer %p\n", replay_buffer.data._data);
                // printf("replay buffer view pointer %p\n", replay_buffer.observations._data);
                // printf("batch pointer %p\n", batch.observations_actions_next_observations._data);
                gather_batch_step<DETERMINISTIC>(device, runner, replay_buffer, batch, batch_step_i, rng_state);
            }
        }

        template<typename DEVICE, typename SPEC, typename RNG>
        __global__
        void prologue_kernel(DEVICE device, rl::components::OffPolicyRunner<SPEC> runner, RNG rng) {
            using TI = typename SPEC::TI;
            // if the episode is done (step limit activated for STEP_LIMIT > 0) or if the step is the first step for this runner, reset the environment
            TI env_i = threadIdx.x + blockIdx.x * blockDim.x;
            static_assert(RNG::NUM_RNGS >= SPEC::PARAMETERS::N_ENVIRONMENTS, "Please increase the number of CUDA RNGs");
            if(env_i < SPEC::PARAMETERS::N_ENVIRONMENTS){
                auto& rng_state = get(rng.states, 0, env_i);
                prologue_per_env(device, runner, rng_state, env_i);
            }
        }
        template<typename DEV_SPEC, typename SPEC, typename RNG>
        void prologue(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, RNG &rng) {
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename SPEC::TI;
            constexpr TI BLOCKSIZE_COLS = 32;
            constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::PARAMETERS::N_ENVIRONMENTS, BLOCKSIZE_COLS);
            dim3 grid(N_BLOCKS_COLS);
            dim3 block(BLOCKSIZE_COLS);
            devices::cuda::TAG<DEVICE, true> tag_device{};
            prologue_kernel<<<grid, block, 0, device.stream>>>(tag_device, runner, rng);
            check_status(device);
        }
        // template<auto POLICY_INDEX, typename DEV_SPEC, typename SPEC, typename POLICY, typename RNG>
        // void interlude(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, POLICY& policy, typename POLICY::template Buffer<SPEC::PARAMETERS::N_ENVIRONMENTS>& policy_eval_buffers, RNG& rng) {
        //     // runner struct should be on the CPU while its buffers should be on the GPU
        //     evaluate(device, policy, runner.buffers.observations, runner.buffers.actions, policy_eval_buffers, rng);
        // }

        template<typename DEVICE, typename SPEC, typename POLICY, typename RNG>
        __global__
        void epilogue_kernel(DEVICE device, rl::components::OffPolicyRunner<SPEC> runner, POLICY policy, RNG rng) {
            using TI = typename SPEC::TI;
            using OFF_POLICY_RUNNER = rl::components::OffPolicyRunner<SPEC>;
            static_assert(RNG::NUM_RNGS >= SPEC::PARAMETERS::N_ENVIRONMENTS, "Please increase the number of CUDA RNGs");
            TI env_i = threadIdx.x + blockIdx.x * blockDim.x;
            static_assert(OFF_POLICY_RUNNER::REPLAY_BUFFER_TYPE::SPEC::DYNAMIC_ALLOCATION == false, "When using the OffPolicyRunner with CUDA, the replay buffers should be statically allocated");
            if(env_i < SPEC::PARAMETERS::N_ENVIRONMENTS){
                POLICY dummy_policy;
                auto& rng_state = get(rng.states, 0, env_i);
                epilogue_per_env(device, runner, dummy_policy, rng_state, env_i);
            }
        }
        template<typename DEV_SPEC, typename SPEC, typename POLICY, typename RNG>
        void epilogue(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, const POLICY& policy, RNG& rng) {
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename SPEC::TI;
            constexpr TI BLOCKSIZE_COLS = 32;
            constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::PARAMETERS::N_ENVIRONMENTS, BLOCKSIZE_COLS);
            dim3 grid(N_BLOCKS_COLS);
            dim3 block(BLOCKSIZE_COLS);
            devices::cuda::TAG<DEVICE, true> tag_device{};
            POLICY dummy_policy; // just for type inference
            epilogue_kernel<<<grid, block, 0, device.stream>>>(tag_device, runner, dummy_policy, rng);
            check_status(device);
        }
    }
    template<typename DEV_SPEC, typename SPEC>
    void malloc(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC> &runner) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        static_assert(!SPEC::DYNAMIC_ALLOCATION_REPLAY_BUFFER, "The replay buffer should not be dynamically allocated when using the OffPolicyRunner using CUDA");
        static_assert(!SPEC::DYNAMIC_ALLOCATION_EPISODE_STATS, "The episode stats should not be dynamically allocated when using the OffPolicyRunner using CUDA");

        malloc(device, runner.buffers);
        malloc(device, runner.envs);
        malloc(device, runner.states);
        malloc(device, runner.env_parameters);
        malloc(device, runner.episode_return);
        malloc(device, runner.episode_step);
        malloc(device, runner.truncated);
        malloc(device, runner.replay_buffers);
        malloc(device, runner.episode_stats);
        malloc(device, runner.policy_states);
    }
    template<typename DEV_SPEC, typename SPEC>
    void free(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC> &runner) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        static_assert(!SPEC::DYNAMIC_ALLOCATION_REPLAY_BUFFER, "The replay buffer should not be dynamically allocated when using the OffPolicyRunner using CUDA");
        static_assert(!SPEC::DYNAMIC_ALLOCATION_EPISODE_STATS, "The episode stats should not be dynamically allocated when using the OffPolicyRunner using CUDA");

        free(device, runner.buffers);
        free(device, runner.envs);
        free(device, runner.states);
        free(device, runner.env_parameters);
        free(device, runner.episode_return);
        free(device, runner.episode_step);
        free(device, runner.truncated);
        free(device, runner.replay_buffers);
        free(device, runner.episode_stats);
        free(device, runner.policy_states);
    }
    template<typename DEV_SPEC, typename SPEC>
    void init(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC> &runner){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::PARAMETERS::N_ENVIRONMENTS, BLOCKSIZE_COLS);
        dim3 grid(N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::init_kernel<<<grid, block, 0, device.stream>>>(tag_device, runner);
        check_status(device);

        truncate_all(device, runner);
        runner.previous_policy_set = false;
        runner.previous_policy = 0;
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        runner.initialized = true;
#endif
    }
    template <typename DEV_SPEC, typename SPEC, typename BATCH_SPEC, typename RNG, bool DETERMINISTIC=false>
    void gather_batch(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, RNG& rng) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        static_assert(utils::typing::is_same_v<SPEC, typename BATCH_SPEC::SPEC>);
        using TI = typename SPEC::TI;
        constexpr typename DEVICE::index_t BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::gather_batch_kernel<DETERMINISTIC><<<bias_grid, bias_block, 0, device.stream>>>(tag_device, runner, batch, rng);
        check_status(device);
    }
    template<auto POLICY_INDEX, typename DEV_SPEC, typename SPEC, typename POLICY, typename POLICY_BUFFERS, typename RNG>
    void step(devices::CUDA<DEV_SPEC>& device, rl::components::OffPolicyRunner<SPEC>& runner, POLICY& policy, POLICY_BUFFERS& policy_eval_buffers, RNG &rng){
        using DEVICE = devices::CUDA<DEV_SPEC>;
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_OFF_POLICY_RUNNER_CHECK_INIT
        utils::assert_exit(device, runner.initialized, "OffPolicyRunner not initialized");
#endif
        // static_assert(POLICY::INPUT_DIM == SPEC::ENVIRONMENT::Observation::DIM, "The policy's input dimension must match the environment's observation dimension.");
        // static_assert(POLICY::OUTPUT_DIM == SPEC::ENVIRONMENT::ACTION_DIM, "The policy's output dimension must match the environment's action dimension.");
        // todo: increase efficiency by removing the double observation of each state

        bool policy_switch = runner.previous_policy_set && (POLICY_INDEX != runner.previous_policy);
        if(policy_switch){
            truncate_all(device, runner);
        }

        rl::components::off_policy_runner::prologue(device, runner, rng);
        rl::components::off_policy_runner::interlude<POLICY_INDEX>(device, runner, policy, policy_eval_buffers, rng);
        rl::components::off_policy_runner::epilogue(device, runner, policy, rng);

        runner.previous_policy = POLICY_INDEX;
        runner.previous_policy_set = true;
    }
    namespace rl::components::off_policy_runner::kernels {
        template <typename DEV_SPEC, typename SPEC>
        __global__
        void update_views(devices::CUDA<DEV_SPEC> device, Matrix<SPEC> replay_buffers) {
            using DEVICE = devices::CUDA<DEV_SPEC>;
            static_assert(SPEC::ROWS == 1);
            using TI = typename DEVICE::index_t;
            TI thread_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(thread_i < SPEC::COLS) {
                auto& rb = get(replay_buffers, 0, thread_i);
                rl_tools::update_views(device, rb);
            }
        }
    }
    template <typename DEV_SPEC, typename SPEC>
    void update_views(devices::CUDA<DEV_SPEC>& device, Matrix<SPEC>& replay_buffers) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        static_assert(SPEC::ROWS == 1);
        using TI = typename DEVICE::index_t;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::COLS, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::kernels::update_views<<<bias_grid, bias_block, 0, device.stream>>>(tag_device, replay_buffers);
        check_status(device);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"


#endif
