#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_EMBEDDING_LAYER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_EMBEDDING_LAYER_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"

//#include "../../../nn/nn.h"
#include "../../../nn/capability/capability.h"
#include "../../../nn/parameters/parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::embedding {
//    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
//    constexpr bool check_input_output_f(){
//        static_assert(length(typename INPUT_SPEC::SHAPE{}) == 1);
//        static_assert(length(typename OUTPUT_SPEC::SHAPE{}) == 2);
//        static_assert(get<0>(typename INPUT_SPEC::SHAPE{}) == get<0>(typename OUTPUT_SPEC::SHAPE{}));
//        static_assert(get<1>(typename OUTPUT_SPEC::SHAPE{}) == LAYER_SPEC::OUTPUT_DIM);
//        return true;
//    }
//    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
//    constexpr bool check_input_output = check_input_output_f<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>();
    template <typename T_TYPE_POLICY, typename T_TI>
    struct StandardNormalSpecification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using T = typename TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        static constexpr T SCALE = 1;
    };
    template<typename SPEC>
    struct StandardNormal {
    };
    template<typename T_T, typename T_TI>
    using DefaultInitializer = StandardNormal<StandardNormalSpecification<T_T, T_TI>>;

    template <typename T_TI>
    struct DefaultInputShapeFactory{
        template <T_TI BATCH_SIZE>
        using SHAPE = tensor::Shape<T_TI, BATCH_SIZE>;
    };
    template<typename T_TYPE_POLICY, typename T_TI, T_TI T_NUM_CLASSES, T_TI T_EMBEDDING_DIM, template <T_TI> typename T_INPUT_SHAPE = DefaultInputShapeFactory<T_TI>::template SHAPE, typename T_INITIALIZER = DefaultInitializer<T_TYPE_POLICY, T_TI>, typename T_PARAMETER_GROUP=parameters::groups::Input>
    struct Configuration {
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr TI NUM_CLASSES = T_NUM_CLASSES;
        static constexpr TI EMBEDDING_DIM = T_EMBEDDING_DIM;
        using INITIALIZER = T_INITIALIZER;
        using PARAMETER_GROUP = T_PARAMETER_GROUP;
        // Summary
        static constexpr TI NUM_WEIGHTS = EMBEDDING_DIM * NUM_CLASSES;
    };

    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using CONFIG = T_CONFIG;
        using CAPABILITY = T_CAPABILITY;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        static_assert(length(INPUT_SHAPE{}) == 3, "The input shape of the Embedding layer must be 3 dimensional for now (sequence x batch x 1 (integer ids))");
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        static constexpr TI SEQUENCE_LENGTH = get<length(INPUT_SHAPE{})-3>(INPUT_SHAPE{});
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = tensor::Replace<NEW_INPUT_SHAPE, CONFIG::EMBEDDING_DIM, length(NEW_INPUT_SHAPE{})-1>;
        using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
        static constexpr TI INTERNAL_BATCH_SIZE = get<1>(INPUT_SHAPE{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
    };

    struct State{};
    struct Buffer{};

    template<typename T_SPEC>
    struct LayerForward {
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI NUM_CLASSES = SPEC::NUM_CLASSES;
        static constexpr TI OUTPUT_DIM = SPEC::EMBEDDING_DIM;
        static constexpr TI NUM_WEIGHTS = SPEC::NUM_WEIGHTS;
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>;
        using WEIGHTS_SHAPE = tensor::Shape<TI, NUM_CLASSES, OUTPUT_DIM>;
        using WEIGHTS_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, WEIGHTS_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Weights, SPEC::DYNAMIC_ALLOCATION>;
        typename SPEC::PARAMETER_TYPE::template Instance<WEIGHTS_PARAMETER_SPEC> weights;

        template<bool DYNAMIC_ALLOCATION=true>
        using State = embedding::State;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = embedding::Buffer;
    };
    template<typename SPEC>
    struct LayerBackward: public LayerForward<SPEC>{
    };
    template<typename SPEC>
    struct LayerGradient: public LayerBackward<SPEC>{
        // This layer supports backpropagation wrt its input but including its weights (for this it stores the intermediate outputs in addition to the pre_activations because they determine the gradient wrt the weights of the following layer)
        using T_ACC = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        using OUTPUT_CONTAINER_SPEC = tensor::Specification<T_ACC, typename SPEC::TI, typename SPEC::OUTPUT_SHAPE, SPEC::DYNAMIC_ALLOCATION>;
        using OUTPUT_CONTAINER_TYPE = Tensor<OUTPUT_CONTAINER_SPEC>;
        OUTPUT_CONTAINER_TYPE output;
    };
    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using Layer =
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward, LayerForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward, LayerBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient, LayerGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

    template <typename CONFIG>
    struct BindConfiguration{
        template <typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = nn::layers::embedding::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
