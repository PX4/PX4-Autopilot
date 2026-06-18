#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_TD3_SAMPLING_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_TD3_SAMPLING_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"
#include "../../../nn/parameters/parameters.h"
#include "../dense/layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    // This layer performs the sampling and squashing for SAC
    namespace nn::layers::td3_sampling{
        namespace mode{
            template <typename T_BASE, typename T_SPEC = bool>
            struct ExternalNoise{
                using SPEC = T_SPEC;
                using BASE = T_BASE;
                // this mode uses the noise from the Buffer for debugging / no-side-effect inference
            };
        }
        template <typename TYPE_POLICY>
        struct DefaultParameters{
            static constexpr typename TYPE_POLICY::DEFAULT STD = 0.1;
        };

        struct State{};

        template <typename T_TI, typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
        struct BufferSpecification {
            using TI = T_TI;
            using SPEC = T_SPEC;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };

        template <typename BUFFER_SPEC>
        struct Buffer{
            using LAYER_SPEC = typename BUFFER_SPEC::SPEC;
            using T = typename LAYER_SPEC::TYPE_POLICY::DEFAULT;
            using NOISE_CONTAINER_SPEC = matrix::Specification<T, typename BUFFER_SPEC::TI, LAYER_SPEC::INTERNAL_BATCH_SIZE, LAYER_SPEC::DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            using NOISE_CONTAINER_TYPE = Matrix<NOISE_CONTAINER_SPEC>;
            NOISE_CONTAINER_TYPE noise;
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
            static constexpr TI DIM = get_last(INPUT_SHAPE{});
            static constexpr TI INPUT_DIM = DIM;
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

            using T = typename TYPE_POLICY::DEFAULT;

            T std = SPEC::PARAMETERS::STD;

            template<bool DYNAMIC_ALLOCATION=true>
            using State = td3_sampling::State;
            template<bool DYNAMIC_ALLOCATION=true>
            using Buffer = td3_sampling::Buffer<td3_sampling::BufferSpecification<TI, SPEC, DYNAMIC_ALLOCATION>>;
        };
        template<typename SPEC>
        struct LayerBackward: public LayerForward<SPEC> {
            using ACTIVATION_T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Activation>;
            using PRE_ACTIVATIONS_CONTAINER_SPEC = matrix::Specification<ACTIVATION_T, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::DIM, SPEC::DYNAMIC_ALLOCATION>;
            using PRE_ACTIVATIONS_CONTAINER_TYPE = Matrix<PRE_ACTIVATIONS_CONTAINER_SPEC>;
            PRE_ACTIVATIONS_CONTAINER_TYPE pre_clip;
        };
        template<typename SPEC>
        struct LayerGradient: public LayerBackward<SPEC> {
            using T = typename SPEC::TYPE_POLICY::DEFAULT;
            using OUTPUT_CONTAINER_SPEC = matrix::Specification<T, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::DIM, SPEC::DYNAMIC_ALLOCATION>;
            using OUTPUT_CONTAINER_TYPE = Matrix<OUTPUT_CONTAINER_SPEC>;
            OUTPUT_CONTAINER_TYPE output;
        };

        template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
        using Layer =
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward,
                LayerForward<layers::td3_sampling::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward,
                LayerBackward<layers::td3_sampling::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient,
                LayerGradient<layers::td3_sampling::Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

        template <typename CONFIG>
        struct BindConfiguration{
            template <typename CAPABILITY, typename INPUT_SHAPE>
            using Layer = nn::layers::td3_sampling::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif