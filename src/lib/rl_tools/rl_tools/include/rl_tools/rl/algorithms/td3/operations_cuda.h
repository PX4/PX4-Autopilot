

#include "../../../utils/polyak/operations_cuda.h"
#include "../../../rl/algorithms/td3/td3.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC, typename SPEC, typename OUTPUT_SPEC, typename RNG>
    __global__
    void target_action_noise_kernel(devices::CUDA<DEV_SPEC>& device, const rl::algorithms::td3::ActorCritic<SPEC> actor_critic, Matrix<OUTPUT_SPEC> target_action_noise, RNG rng ) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI BATCH_SIZE = SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
        curandState rng_state;

        TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
        curand_init(rng, batch_step_i, 0, &rng_state);
        if(batch_step_i < BATCH_SIZE){
            for(TI action_i=0; action_i < SPEC::ENVIRONMENT::ACTION_DIM; action_i++){
                set(target_action_noise, batch_step_i, action_i, math::clamp(device.math,
                    random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T)0, SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_STD, rng_state),
//                        curand_normal(&rng_state),
                        -(T)SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_CLIP,
                         (T)SPEC::PARAMETERS::TARGET_NEXT_ACTION_NOISE_CLIP
                ));
            }
        }
    }
    template <typename DEV_SPEC, typename SPEC, typename OUTPUT_SPEC, typename RNG>
    void target_action_noise(devices::CUDA<DEV_SPEC>& device, const rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, Matrix<OUTPUT_SPEC>& target_action_noise, RNG& rng ) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        static_assert(OUTPUT_SPEC::ROWS == SPEC::PARAMETERS::CRITIC_BATCH_SIZE);
        static_assert(OUTPUT_SPEC::COLS == SPEC::ENVIRONMENT::ACTION_DIM);
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI BATCH_SIZE = SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        target_action_noise_kernel<DEV_SPEC, SPEC, OUTPUT_SPEC, RNG><<<bias_grid, bias_block, 0, device.stream>>>(device, actor_critic, target_action_noise, rng);
        check_status(device);
    }

    template <typename DEV_SPEC, typename SPEC>
    __global__
    void noisy_next_actions_kernel(devices::CUDA<DEV_SPEC>& device, rl::algorithms::td3::CriticTrainingBuffers<SPEC> training_buffers) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        using BUFFERS = rl::algorithms::td3::CriticTrainingBuffers<SPEC>;
        constexpr TI BATCH_SIZE = BUFFERS::BATCH_SIZE;
        TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
        if(batch_step_i < BATCH_SIZE){
            for(TI action_i=0; action_i < SPEC::ENVIRONMENT::ACTION_DIM; action_i++){
                T noisy_next_action = get(training_buffers.next_actions, batch_step_i, action_i) + get(training_buffers.target_next_action_noise, batch_step_i, action_i);
                noisy_next_action = math::clamp<T>(device.math, noisy_next_action, -1, 1);
                set(training_buffers.next_actions, batch_step_i, action_i, noisy_next_action);
            }
        }
    }
    template <typename DEV_SPEC, typename SPEC>
    void noisy_next_actions(devices::CUDA<DEV_SPEC>& device, rl::algorithms::td3::CriticTrainingBuffers<SPEC> training_buffers) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI BATCH_SIZE = SPEC::PARAMETERS::CRITIC_BATCH_SIZE;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        noisy_next_actions_kernel<DEV_SPEC, SPEC><<<bias_grid, bias_block, 0, device.stream>>>(device, training_buffers);
        check_status(device);
    }

    template <typename DEV_SPEC, typename OFF_POLICY_RUNNER_SPEC, auto BATCH_SIZE, typename SPEC>
    __global__
    void target_actions_kernel(devices::CUDA<DEV_SPEC>& device, rl::components::off_policy_runner::Batch<rl::components::off_policy_runner::BatchSpecification<OFF_POLICY_RUNNER_SPEC, BATCH_SIZE>> batch, rl::algorithms::td3::CriticTrainingBuffers<SPEC> training_buffers, typename SPEC::T gamma) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        using BUFFERS = rl::algorithms::td3::CriticTrainingBuffers<SPEC>;
        static_assert(BATCH_SIZE == BUFFERS::BATCH_SIZE);
        TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
        if(batch_step_i < BATCH_SIZE){
            T min_next_state_action_value = math::min(device.math,
                    get(training_buffers.next_state_action_value_critic_1, batch_step_i, 0),
                    get(training_buffers.next_state_action_value_critic_2, batch_step_i, 0)
            );
            T reward = get(batch.rewards, 0, batch_step_i);
            bool terminated = get(batch.terminated, 0, batch_step_i);
            T future_value = SPEC::PARAMETERS::IGNORE_TERMINATION || !terminated ? gamma * min_next_state_action_value : 0;
            T current_target_action_value = reward + future_value;
            set(training_buffers.target_action_value, batch_step_i, 0, current_target_action_value); // todo: improve pitch of target action values etc. (by transformig it into row vectors instead of column vectors)
        }
    }
    template <typename DEV_SPEC, typename OFF_POLICY_RUNNER_SPEC, auto BATCH_SIZE, typename SPEC>
    void target_action_values(devices::CUDA<DEV_SPEC>& device, const rl::algorithms::td3::ActorCritic<SPEC>& actor_critic, rl::components::off_policy_runner::Batch<rl::components::off_policy_runner::BatchSpecification<OFF_POLICY_RUNNER_SPEC, BATCH_SIZE>> batch, rl::algorithms::td3::CriticTrainingBuffers<SPEC> training_buffers) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        target_actions_kernel<<<bias_grid, bias_block, 0, device.stream>>>(device, batch, training_buffers, actor_critic.gamma);
        check_status(device);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"
