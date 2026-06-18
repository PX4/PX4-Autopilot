#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MLP_NETWORK_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MLP_NETWORK_H

#include "../../nn/nn.h"
#include "../../numeric_types/categories.h"
#include "../../nn/parameters/parameters.h"
#include "../../nn/layers/dense/layer.h"
#include "../../nn/optimizers/sgd/sgd.h"
#include "../../nn/optimizers/adam/adam.h"
#include "../../utils/generic/typing.h"
#include "../../containers/matrix/matrix.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::mlp {
    template <typename T_TYPE_POLICY, typename T_TI, T_TI T_OUTPUT_DIM, T_TI T_NUM_LAYERS, T_TI T_HIDDEN_DIM, nn::activation_functions::ActivationFunction T_HIDDEN_ACTIVATION_FUNCTION, nn::activation_functions::ActivationFunction T_OUTPUT_ACTIVATION_FUNCTION, typename T_LAYER_INITIALIZER=nn::layers::dense::DefaultInitializer<T_TYPE_POLICY, T_TI>, bool T_IS_INPUT=true, bool T_IS_OUTPUT=true>
    struct Configuration{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr T_TI OUTPUT_DIM = T_OUTPUT_DIM;
        static constexpr T_TI NUM_LAYERS = T_NUM_LAYERS; // The input and output layers count towards the total number of layers
        static_assert(NUM_LAYERS >= 2); // At least input and output layer are required
        static constexpr TI NUM_HIDDEN_LAYERS = NUM_LAYERS - 2;
        static constexpr T_TI HIDDEN_DIM = T_HIDDEN_DIM;
        static constexpr auto HIDDEN_ACTIVATION_FUNCTION = T_HIDDEN_ACTIVATION_FUNCTION;
        static constexpr auto OUTPUT_ACTIVATION_FUNCTION = T_OUTPUT_ACTIVATION_FUNCTION;
        static constexpr TI IS_INPUT = T_IS_INPUT;
        static constexpr TI IS_OUTPUT = T_IS_OUTPUT;

        using LAYER_INITIALIZER = T_LAYER_INITIALIZER;

    };
    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using CONFIG = T_CONFIG;
        using CAPABILITY = T_CAPABILITY;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = tensor::Replace<NEW_INPUT_SHAPE, CONFIG::OUTPUT_DIM, length(NEW_INPUT_SHAPE{})-1>;
        using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
        static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<INPUT_SHAPE>>{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
        static constexpr TI NUM_WEIGHTS = CONFIG::OUTPUT_DIM * INPUT_DIM + CONFIG::OUTPUT_DIM;

        using INPUT_LAYER_CONFIG  = nn::layers::dense::Configuration<TYPE_POLICY, TI, CONFIG::HIDDEN_DIM, CONFIG::HIDDEN_ACTIVATION_FUNCTION, typename CONFIG::LAYER_INITIALIZER, utils::typing::conditional_t<CONFIG::IS_INPUT, nn::parameters::groups::Input, nn::parameters::groups::Normal>>;
        using HIDDEN_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, CONFIG::HIDDEN_DIM, CONFIG::HIDDEN_ACTIVATION_FUNCTION, typename CONFIG::LAYER_INITIALIZER, nn::parameters::groups::Normal>;
        using OUTPUT_LAYER_CONFIG = nn::layers::dense::Configuration<TYPE_POLICY, TI, CONFIG::OUTPUT_DIM, CONFIG::OUTPUT_ACTIVATION_FUNCTION, typename CONFIG::LAYER_INITIALIZER, utils::typing::conditional_t<CONFIG::IS_OUTPUT ,nn::parameters::groups::Output, nn::parameters::groups::Normal>>;
        using INPUT_LAYER = nn::layers::dense::Layer<INPUT_LAYER_CONFIG, CAPABILITY, INPUT_SHAPE>;
        using HIDDEN_LAYER = nn::layers::dense::Layer<HIDDEN_LAYER_CONFIG, CAPABILITY, typename INPUT_LAYER::SPEC::OUTPUT_SHAPE>;
        using OUTPUT_LAYER = nn::layers::dense::Layer<OUTPUT_LAYER_CONFIG, CAPABILITY, typename HIDDEN_LAYER::SPEC::OUTPUT_SHAPE>;
    };

    template<typename SPEC_1, typename SPEC_2>
    constexpr bool check_spec_memory =
            // utils::typing::is_same_v<typename SPEC_1::T, typename SPEC_2::T>
            SPEC_1::INPUT_DIM == SPEC_2::INPUT_DIM
            && SPEC_1::OUTPUT_DIM == SPEC_2::OUTPUT_DIM
            && SPEC_1::NUM_LAYERS == SPEC_2::NUM_LAYERS
            && SPEC_1::HIDDEN_DIM == SPEC_2::HIDDEN_DIM;
    template<typename SPEC_1, typename SPEC_2>
    constexpr bool check_spec =
            check_spec_memory<SPEC_1, SPEC_2>
            && SPEC_1::HIDDEN_ACTIVATION_FUNCTION == SPEC_2::HIDDEN_ACTIVATION_FUNCTION
            && SPEC_1::OUTPUT_ACTIVATION_FUNCTION == SPEC_2::OUTPUT_ACTIVATION_FUNCTION;

//    template <typename T_CAPABILITY, typename T_SPEC>
//    struct CapabilitySpecification: T_SPEC, T_CAPABILITY{
//        using CAPABILITY = T_CAPABILITY;
//        using PARAMETER_TYPE = typename CAPABILITY::PARAMETER_TYPE;
//    };
    struct State{};

    // T_LAYER_PROTOTYPE is any of INPUT_LAYER, HIDDEN_LAYER, or OUTPUT_LAYER. It is assumed that the buffer for all of them are the same (likely empty)
    template<typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
    struct NeuralNetworkBuffersSpecification{
        using SPEC = T_SPEC;
        using TI = typename SPEC::TI;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        static constexpr TI INTERNAL_BATCH_SIZE = SPEC::INTERNAL_BATCH_SIZE;
        static constexpr TI DIM = SPEC::HIDDEN_DIM;
    };

    struct BuffersTag {
        using CATEGORY = numeric_types::categories::Accumulator;
    };

    template<typename T_BUFFER_SPEC>
    struct NeuralNetworkBuffers{
        using BUFFER_SPEC = T_BUFFER_SPEC;
        using MLP_SPEC = typename BUFFER_SPEC::SPEC;
        using TYPE_POLICY = typename MLP_SPEC::TYPE_POLICY;
        using TI = typename MLP_SPEC::TI;
        static constexpr TI INTERNAL_BATCH_SIZE = T_BUFFER_SPEC::INTERNAL_BATCH_SIZE;
        using T = typename MLP_SPEC::TYPE_POLICY::template GET<BuffersTag>;
        using TICK_TOCK_CONTAINER_SPEC = matrix::Specification<T, TI, INTERNAL_BATCH_SIZE, BUFFER_SPEC::DIM, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
        using TICK_TOCK_CONTAINER_TYPE = Matrix<TICK_TOCK_CONTAINER_SPEC>;
        TICK_TOCK_CONTAINER_TYPE tick;
        TICK_TOCK_CONTAINER_TYPE tock;
        using LayerBuffer = typename MLP_SPEC::INPUT_LAYER::template Buffer<BUFFER_SPEC::DYNAMIC_ALLOCATION>;
        LayerBuffer layer_buffer;
    };

    template<typename T_SPEC>
    struct NeuralNetworkForward{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        // Could be dependent on the capability but in this case the buffer-requirements of forward and backward are the same

        // Convenience
        static_assert(SPEC::NUM_LAYERS >= 2); // At least input and output layer are required
        static constexpr TI NUM_HIDDEN_LAYERS = SPEC::NUM_LAYERS - 2;
        static_assert(SPEC::NUM_HIDDEN_LAYERS == NUM_HIDDEN_LAYERS);

        // Interface
        // static constexpr TI  INPUT_DIM = SPEC::INPUT_LAYER_SPEC::INPUT_DIM;
        // static constexpr TI OUTPUT_DIM = SPEC::OUTPUT_LAYER_SPEC::OUTPUT_DIM;
        static constexpr TI NUM_WEIGHTS = SPEC::INPUT_LAYER::NUM_WEIGHTS + SPEC::HIDDEN_LAYER::NUM_WEIGHTS * NUM_HIDDEN_LAYERS + SPEC::OUTPUT_LAYER::NUM_WEIGHTS;

        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>;

        // Storage
        typename SPEC::INPUT_LAYER input_layer;
        typename SPEC::HIDDEN_LAYER  hidden_layers[NUM_HIDDEN_LAYERS];
        typename SPEC::OUTPUT_LAYER  output_layer;

        template<bool DYNAMIC_ALLOCATION=true>
        using State = mlp::State;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = NeuralNetworkBuffers<NeuralNetworkBuffersSpecification<SPEC, DYNAMIC_ALLOCATION>>;
    };

    template<typename SPEC>
    struct NeuralNetworkBackward: public NeuralNetworkForward<SPEC>{
//        static constexpr typename SPEC::TI BATCH_SIZE = SPEC::BATCH_SIZE;
    };
    template<typename SPEC>
    struct NeuralNetworkGradient: public NeuralNetworkBackward<SPEC>{};

    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using NeuralNetwork =
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward, NeuralNetworkForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward, NeuralNetworkBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient, NeuralNetworkGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

    template <typename T_CONFIG>
    struct BindConfiguration{
        template <typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = nn_models::mlp::NeuralNetwork<T_CONFIG, CAPABILITY, INPUT_SHAPE>;
    };

    template <typename CAPABILITY, typename MODULE, typename INPUT_SHAPE>
    struct Build: NeuralNetwork<CAPABILITY, MODULE, INPUT_SHAPE>{
        template <typename TI, TI BATCH_SIZE>
        struct CHANGE_BATCH_SIZE_IMPL{
            using NEW_INPUT_SHAPE = tensor::Replace<INPUT_SHAPE, BATCH_SIZE, 1>;
            using CHANGE_BATCH_SIZE = Build<CAPABILITY, MODULE, NEW_INPUT_SHAPE>;
        };
        template <typename TI, TI BATCH_SIZE>
        using CHANGE_BATCH_SIZE = typename CHANGE_BATCH_SIZE_IMPL<TI, BATCH_SIZE>::CHANGE_BATCH_SIZE;
        template <typename NEW_CAPABILITY>
        using CHANGE_CAPABILITY = Build<NEW_CAPABILITY, MODULE, INPUT_SHAPE>;
    };


}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
