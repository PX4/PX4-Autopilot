

#include "../../../utils/polyak/operations_cuda.h"
#include "../../../rl/algorithms/sac/sac.h"
#include "../../../rl/components/off_policy_runner/off_policy_runner.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::components::off_policy_runner::kernels{
        // template <typename DEV_SPEC, typename OFF_POLICY_RUNNER_SPEC, auto BATCH_SIZE, typename SPEC, typename NEXT_ACTION_LOG_PROBS_SPEC,typename ALPHA_PARAMETER>
        template <typename DEV_SPEC, typename BATCH_SPEC, typename TRAINING_BUFFER_SPEC, typename NEXT_ACTION_LOG_PROBS_SPEC, typename LOG_ALPHA_SPEC>
        __global__
        void target_action_values(devices::CUDA<DEV_SPEC> device, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC> batch, rl::algorithms::sac::CriticTrainingBuffers<TRAINING_BUFFER_SPEC> training_buffers, const Matrix<NEXT_ACTION_LOG_PROBS_SPEC> next_action_log_probs, Tensor<LOG_ALPHA_SPEC> log_alpha){
        // void target_action_values_kernel(devices::CUDA<DEV_SPEC> device, rl::components::off_policy_runner::SequentialBatch<rl::components::off_policy_runner::SequentialBatchSpecification<OFF_POLICY_RUNNER_SPEC, BATCH_SIZE>> batch, rl::algorithms::sac::CriticTrainingBuffers<SPEC> training_buffers, const Matrix<NEXT_ACTION_LOG_PROBS_SPEC> next_action_log_probs, ALPHA_PARAMETER log_alpha) {
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using T = typename BATCH_SPEC::TYPE_POLICY::DEFAULT;
            using TI = typename DEVICE::index_t;
            using BUFFERS = rl::algorithms::sac::CriticTrainingBuffers<TRAINING_BUFFER_SPEC>;
            using BATCH = rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>;
            constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
            constexpr TI SEQUENCE_LENGTH = BATCH_SPEC::SEQUENCE_LENGTHH;
            constexpr TI N_VALUES = BATCH_SIZE * SEQUENCE_LENGTH;
            static_assert(BATCH_SIZE == BUFFERS::BATCH_SIZE);
            T alpha = math::exp(typename DEVICE::SPEC::MATH{}, get(device, log_alpha, 0));
            TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(batch_step_i < N_VALUES){
                target_action_values_per_sample(device, batch, training_buffers, next_action_log_probs, alpha, batch_step_i);
            }
        }
    }
    template <typename DEV_SPEC, typename BATCH_SPEC, typename TRAINING_BUFFER_SPEC, typename NEXT_ACTION_LOG_PROBS_SPEC, typename LOG_ALPHA_PARAMETER>
    void target_action_values(devices::CUDA<DEV_SPEC>& device, rl::components::off_policy_runner::SequentialBatch<BATCH_SPEC>& batch, rl::algorithms::sac::CriticTrainingBuffers<TRAINING_BUFFER_SPEC>& training_buffers, const Matrix<NEXT_ACTION_LOG_PROBS_SPEC>& next_action_log_probs, LOG_ALPHA_PARAMETER& log_alpha){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename BATCH_SPEC::SPEC::TI;
        constexpr TI BATCH_SIZE = BATCH_SPEC::BATCH_SIZE;
        constexpr TI SEQUENCE_LENGTH = BATCH_SPEC::SEQUENCE_LENGTHH;
        constexpr TI N_VALUES = BATCH_SIZE * SEQUENCE_LENGTH;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(N_VALUES, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::kernels::target_action_values<<<bias_grid, bias_block, 0, device.stream>>>(tag_device, batch, training_buffers, next_action_log_probs, log_alpha.parameters);
        check_status(device);
    }

    template <typename DEVICE, typename SPEC, typename TRAINING_BUFFERS_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void min_value_d_output_per_sample(DEVICE& device, rl::algorithms::sac::ActorCritic<SPEC>& actor_critic, rl::algorithms::sac::ActorTrainingBuffers<TRAINING_BUFFERS_SPEC>& training_buffers, typename DEVICE::index_t batch_i);
    namespace rl::components::off_policy_runner::kernels {
        template <typename DEV_SPEC, typename SPEC, typename TRAINING_BUFFERS_SPEC>
        __global__
        void min_value_d_output_kernel(devices::CUDA<DEV_SPEC> device, rl::algorithms::sac::ActorCritic<SPEC> actor_critic, rl::algorithms::sac::ActorTrainingBuffers<TRAINING_BUFFERS_SPEC> training_buffers){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename DEVICE::index_t;
            using BUFFERS = rl::algorithms::sac::ActorTrainingBuffers<TRAINING_BUFFERS_SPEC>;
            constexpr TI BATCH_SIZE = BUFFERS::BATCH_SIZE;
            TI batch_step_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(batch_step_i < BATCH_SIZE){
                min_value_d_output_per_sample(device, actor_critic, training_buffers, batch_step_i);
            }
        }
    }
    template <typename DEV_SPEC, typename SPEC, typename TRAINING_BUFFERS_SPEC>
    void min_value_d_output(devices::CUDA<DEV_SPEC>& device, rl::algorithms::sac::ActorCritic<SPEC>& actor_critic, rl::algorithms::sac::ActorTrainingBuffers<TRAINING_BUFFERS_SPEC>& training_buffers) {
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename SPEC::TI;
        constexpr TI BATCH_SIZE = rl::algorithms::sac::ActorTrainingBuffers<TRAINING_BUFFERS_SPEC>::BATCH_SIZE;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(BATCH_SIZE, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::kernels::min_value_d_output_kernel<<<bias_grid, bias_block, 0, device.stream>>>(tag_device, actor_critic, training_buffers);
        check_status(device);
    }
    namespace rl::components::off_policy_runner::kernels{
        template <typename DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC, typename MASK_SPEC>
        __global__
        void mask_actions(devices::CUDA<DEV_SPEC> device, Tensor<SOURCE_SPEC> source, Tensor<TARGET_SPEC> target, Tensor<MASK_SPEC> mask, bool invert_mask=false){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename DEVICE::index_t;
            constexpr TI SEQUENCE_LENGTH = get<0>(typename SOURCE_SPEC::SHAPE{});
            TI seq_step_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(seq_step_i < SEQUENCE_LENGTH){
                mask_actions_step(device, source, target, mask, seq_step_i, invert_mask);
            }
        }
    }
    template <typename DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC, typename MASK_SPEC>
    void mask_actions(devices::CUDA<DEV_SPEC>& device, Tensor<SOURCE_SPEC>& source, Tensor<TARGET_SPEC>& target, Tensor<MASK_SPEC>& mask, bool invert_mask=false){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SOURCE_SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI SEQUENCE_LENGTH = get<0>(typename SOURCE_SPEC::SHAPE{});
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SEQUENCE_LENGTH, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::kernels::mask_actions<<<bias_grid, bias_block, 0, device.stream>>>(tag_device, source, target, mask, invert_mask);
        check_status(device);
    }
    namespace rl::components::off_policy_runner::kernels{
        template <typename DEV_SPEC, typename SOURCE_SPEC, typename MASK_SPEC>
        __global__
        void mask_gradient(devices::CUDA<DEV_SPEC> device, Tensor<SOURCE_SPEC> source, Tensor<MASK_SPEC> mask, bool invert_mask=false){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using TI = typename DEVICE::index_t;
            constexpr TI SEQUENCE_LENGTH = get<0>(typename SOURCE_SPEC::SHAPE{});
            TI seq_step_i = threadIdx.x + blockIdx.x * blockDim.x;
            if(seq_step_i < SEQUENCE_LENGTH){
                mask_gradient_step(device, source, mask, seq_step_i, invert_mask);
            }
        }
    }
    template <typename DEV_SPEC, typename SOURCE_SPEC, typename MASK_SPEC>
    void mask_gradient(devices::CUDA<DEV_SPEC>& device, Tensor<SOURCE_SPEC>& source,Tensor<MASK_SPEC>& mask, bool invert_mask=false){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using T = typename SOURCE_SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI SEQUENCE_LENGTH = get<0>(typename SOURCE_SPEC::SHAPE{});
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SEQUENCE_LENGTH, BLOCKSIZE_COLS);
        dim3 bias_grid(N_BLOCKS_COLS);
        dim3 bias_block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        rl::components::off_policy_runner::kernels::mask_gradient<<<bias_grid, bias_block, 0, device.stream>>>(tag_device, source, mask, invert_mask);
        check_status(device);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#include "operations_generic.h"
