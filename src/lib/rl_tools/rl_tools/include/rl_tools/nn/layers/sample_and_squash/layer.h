#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_LAYER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_LAYER_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"
#include "../../../nn/parameters/parameters.h"
#include "../dense/layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    // This layer performs the sampling and squashing for SAC
    namespace nn::layers::sample_and_squash{
        namespace mode{
            template <typename T_BASE, typename T_SPEC = bool>
            struct ExternalNoise{
                using SPEC = T_SPEC;
                using BASE = T_BASE;
                // this mode uses the noise from the Buffer for debugging / no-side-effect inference
            };
            template <typename T_BASE, typename T_SPEC = bool>
            struct DisableEntropy{
                using SPEC = T_SPEC;
                using BASE = T_BASE;
                // this mode uses the noise from the Buffer for debugging / no-side-effect inference
            };
        }
        template <typename TYPE_POLICY>
        struct DefaultParameters{
            using T = typename TYPE_POLICY::DEFAULT;
            static constexpr T LOG_STD_LOWER_BOUND = -20;
            static constexpr T LOG_STD_UPPER_BOUND = 2;
            static constexpr T LOG_PROBABILITY_EPSILON = 1e-6;
            static constexpr bool ADAPTIVE_ALPHA = true;
            static constexpr bool UPDATE_ALPHA_WITH_ACTOR = false;
            static constexpr T ALPHA = 1.0;
            static constexpr T TARGET_ENTROPY = -1;
        };

        struct State{};

        template <typename T_TI, typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
        struct BufferSpecification{
            using TI = T_TI;
            using SPEC = T_SPEC;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };

        template <typename BUFFER_SPEC>
        struct Buffer{
            using T_BUFFER = typename BUFFER_SPEC::SPEC::TYPE_POLICY::template GET<numeric_types::categories::Buffer>;
            using LAYER_SPEC = typename BUFFER_SPEC::SPEC;
            using NOISE_CONTAINER_SPEC = matrix::Specification<T_BUFFER, typename BUFFER_SPEC::TI, LAYER_SPEC::INTERNAL_BATCH_SIZE, LAYER_SPEC::DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            using NOISE_CONTAINER_TYPE = Matrix<NOISE_CONTAINER_SPEC>;
            NOISE_CONTAINER_TYPE noise;

            using LOG_PROBABILITIES_CONTAINER_SPEC = matrix::Specification<T_BUFFER, typename LAYER_SPEC::TI, 1, LAYER_SPEC::INTERNAL_BATCH_SIZE, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            using LOG_PROBABILITIES_CONTAINER_TYPE = Matrix<LOG_PROBABILITIES_CONTAINER_SPEC>;
            LOG_PROBABILITIES_CONTAINER_TYPE log_probabilities;

            using D_LOG_ALPHA_CONTAINER_SPEC = matrix::Specification<T_BUFFER, typename BUFFER_SPEC::TI, 1, LAYER_SPEC::INTERNAL_BATCH_SIZE, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            using D_LOG_ALPHA_CONTAINER_TYPE = Matrix<D_LOG_ALPHA_CONTAINER_SPEC>;
            D_LOG_ALPHA_CONTAINER_TYPE d_log_alpha;
        };
        template<typename T_TYPE_POLICY, typename T_TI, typename T_PARAMETERS = DefaultParameters<T_TYPE_POLICY>>
        struct Configuration {
            using TYPE_POLICY = T_TYPE_POLICY;
            using TI = T_TI;
            using PARAMETERS = T_PARAMETERS;
        };
        template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
        struct Specification: T_CAPABILITY, T_CONFIG{
            using CONFIG = T_CONFIG;
            using CAPABILITY = T_CAPABILITY;
            using INPUT_SHAPE = T_INPUT_SHAPE;
            using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
            using TI = typename CONFIG::TI;
            static constexpr TI DIM_2X = get_last(INPUT_SHAPE{});
            static_assert(DIM_2X % 2 == 0, "Sample and squash layer: The dimension of the input shape must be divisible by 2.");
            static constexpr TI DIM = DIM_2X/2;
            static constexpr TI INPUT_DIM = DIM_2X;
            static constexpr TI OUTPUT_DIM = DIM;
            template <typename NEW_INPUT_SHAPE>
            using OUTPUT_SHAPE_FACTORY = tensor::Replace<NEW_INPUT_SHAPE, OUTPUT_DIM, length(NEW_INPUT_SHAPE{})-1>;
            using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
            static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<INPUT_SHAPE>>{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
            static constexpr TI NUM_WEIGHTS = 0;
        };
        template <typename T_SPEC>
        struct LayerForward{
            using SPEC = T_SPEC;
            using TYPE_POLICY = typename SPEC::TYPE_POLICY;
            using TI = typename SPEC::TI;
            static constexpr TI DIM = SPEC::DIM;
            static constexpr TI INPUT_DIM = SPEC::INPUT_DIM;
            static constexpr TI OUTPUT_DIM = SPEC::OUTPUT_DIM;
            static constexpr TI INTERNAL_BATCH_SIZE = SPEC::INTERNAL_BATCH_SIZE;
            using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
            using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
            template <typename NEW_INPUT_SHAPE>
            using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>;
//            static constexpr TI BATCH_SIZE = SPEC::BATCH_SIZE;
//            using INPUT_SHAPE = typename SPEC::INPUT_SHAPE_FACTORY::template SHAPE<TI, BATCH_SIZE, INPUT_DIM>;
//            using OUTPUT_SHAPE = tensor::Replace<INPUT_SHAPE, OUTPUT_DIM, length(INPUT_SHAPE{})-1>;
//            static constexpr TI ACTUAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<OUTPUT_SHAPE>>{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the ACTUAL_BATCH_SIZE accounts for all leading dimensions.
            template<bool DYNAMIC_ALLOCATION=true>
            using State = sample_and_squash::State;
            template<bool DYNAMIC_ALLOCATION=true>
            using Buffer = sample_and_squash::Buffer<sample_and_squash::BufferSpecification<TI, SPEC, DYNAMIC_ALLOCATION>>;
        };
        template<typename SPEC>
        struct LayerBackward: public LayerForward<SPEC> {
//            static constexpr typename SPEC::TI ACTUAL_BATCH_SIZE = LayerForward<SPEC>::ACTUAL_BATCH_SIZE;
            // This layer supports backpropagation wrt its input but not its weights (for this it stores the intermediate pre_activations)
            using ACTIVATION_TYPE = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Activation>;
            using PRE_ACTIVATIONS_CONTAINER_SPEC = matrix::Specification<ACTIVATION_TYPE, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::DIM, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
            using PRE_ACTIVATIONS_CONTAINER_TYPE = Matrix<PRE_ACTIVATIONS_CONTAINER_SPEC>;
            PRE_ACTIVATIONS_CONTAINER_TYPE pre_squashing, noise;
        };
        template<typename SPEC>
        struct LayerGradient: public LayerBackward<SPEC> {
            using TI = typename SPEC::TI;
//            static constexpr typename SPEC::TI ACTUAL_BATCH_SIZE = LayerBackward<SPEC>::ACTUAL_BATCH_SIZE;
            using ACTIVATION_TYPE = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Activation>;
            using LOG_PROBABILITIES_CONTAINER_SPEC = matrix::Specification<ACTIVATION_TYPE, typename SPEC::TI, 1, SPEC::INTERNAL_BATCH_SIZE, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
            using LOG_PROBABILITIES_CONTAINER_TYPE = Matrix<LOG_PROBABILITIES_CONTAINER_SPEC>;
            LOG_PROBABILITIES_CONTAINER_TYPE log_probabilities;
            using OUTPUT_CONTAINER_SPEC = matrix::Specification<ACTIVATION_TYPE, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::DIM, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
            using OUTPUT_CONTAINER_TYPE = Matrix<OUTPUT_CONTAINER_SPEC>;
            OUTPUT_CONTAINER_TYPE output;
            using ALPHA_SHAPE = tensor::Shape<TI, 1>;
            using ALPHA_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<typename SPEC::TYPE_POLICY, TI, ALPHA_SHAPE, nn::parameters::categories::Biases, nn::parameters::groups::Normal, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
            typename SPEC::PARAMETER_TYPE::template Instance<ALPHA_PARAMETER_SPEC> log_alpha;
        };
        template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
        using Layer =
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward,
                LayerForward<layers::sample_and_squash::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward,
                LayerBackward<layers::sample_and_squash::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient,
                LayerGradient<layers::sample_and_squash::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

        template <typename CONFIG>
        struct BindConfiguration{
            template <typename CAPABILITY, typename INPUT_SHAPE>
            using Layer = nn::layers::sample_and_squash::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif