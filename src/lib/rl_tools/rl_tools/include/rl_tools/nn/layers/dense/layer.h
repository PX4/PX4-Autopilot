#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_LAYER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_LAYER_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"

//#include "../../../nn/nn.h"
#include "../../../nn/capability/capability.h"
#include "../../../nn/parameters/parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::dense {
    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    constexpr bool check_input_output_f(){
        static_assert(INPUT_SPEC::COLS == LAYER_SPEC::INPUT_DIM);
        static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
        //                INPUT_SPEC::ROWS <= OUTPUT_SPEC::ROWS && // todo: could be relaxed to not fill the full output
        static_assert(OUTPUT_SPEC::COLS == LAYER_SPEC::OUTPUT_DIM);
        // static_assert(utils::typing::is_same_v<typename LAYER_SPEC::T, typename INPUT_SPEC::T>);
        // static_assert(utils::typing::is_same_v<typename INPUT_SPEC::T, typename OUTPUT_SPEC::T>);
        return true;
    }
    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    constexpr bool check_input_output = check_input_output_f<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>();

    template <typename T_TYPE_POLICY, typename T_TI>
    struct KaimingUniformSpecification{
        using T = typename T_TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        static constexpr bool INIT_LEGACY = true;
        static constexpr T SCALE = 1;
    };
    template<typename SPEC>
    struct KaimingUniform {
    };
    template<typename T_TYPE_POLICY, typename T_TI>
    using DefaultInitializer = KaimingUniform<KaimingUniformSpecification<T_TYPE_POLICY, T_TI>>;


    template<typename T_TYPE_POLICY, typename T_TI, T_TI T_OUTPUT_DIM, nn::activation_functions::ActivationFunction T_ACTIVATION_FUNCTION, typename T_INITIALIZER = DefaultInitializer<T_TYPE_POLICY, T_TI>, typename T_PARAMETER_GROUP=parameters::groups::Normal>
    struct Configuration{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr TI OUTPUT_DIM = T_OUTPUT_DIM;
        static constexpr nn::activation_functions::ActivationFunction ACTIVATION_FUNCTION = T_ACTIVATION_FUNCTION;
        using INITIALIZER = T_INITIALIZER;
        using PARAMETER_GROUP = T_PARAMETER_GROUP;
    };
    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using CONFIG = T_CONFIG;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        using CAPABILITY = T_CAPABILITY;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        template <typename NEW_INPUT_SHAPE>
        struct OUTPUT_SHAPE_FACTORY{
            static constexpr TI NEW_INPUT_DIM = get_last(NEW_INPUT_SHAPE{});
            static_assert(NEW_INPUT_DIM == INPUT_DIM);
            using SHAPE = tensor::Replace<NEW_INPUT_SHAPE, CONFIG::OUTPUT_DIM, length(NEW_INPUT_SHAPE{})-1>;
        };
        using OUTPUT_SHAPE = typename OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>::SHAPE;
        static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<INPUT_SHAPE>>{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
        static constexpr TI NUM_WEIGHTS = CONFIG::OUTPUT_DIM * INPUT_DIM + CONFIG::OUTPUT_DIM;
    };
    template<typename SPEC_1, typename SPEC_2>
    constexpr bool check_spec_memory =
            // utils::typing::is_same_v<typename SPEC_1::T, typename SPEC_2::T>
            SPEC_1::INPUT_DIM == SPEC_2::INPUT_DIM
            && SPEC_1::OUTPUT_DIM == SPEC_2::OUTPUT_DIM;
    template<typename SPEC_1, typename SPEC_2>
    constexpr bool check_spec =
        check_spec_memory<SPEC_1, SPEC_2>
        && SPEC_1::ACTIVATION_FUNCTION == SPEC_2::ACTIVATION_FUNCTION;

    struct State{};
    struct Buffer{};

    template<typename T_SPEC>
    struct LayerForward {
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI INPUT_DIM = SPEC::INPUT_DIM;
        static constexpr TI OUTPUT_DIM = SPEC::OUTPUT_DIM;
        static constexpr TI NUM_WEIGHTS = SPEC::NUM_WEIGHTS;
        static constexpr TI INTERNAL_BATCH_SIZE = SPEC::INTERNAL_BATCH_SIZE;
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>::SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        using WEIGHTS_SHAPE = tensor::Shape<TI, OUTPUT_DIM, INPUT_DIM>;
        using WEIGHTS_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, WEIGHTS_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Weights, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<WEIGHTS_PARAMETER_SPEC> weights;


        using BIASES_SHAPE = tensor::Shape<TI, OUTPUT_DIM>;
        using BIASES_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, BIASES_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Biases, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<BIASES_PARAMETER_SPEC> biases;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = dense::Buffer;
        template<bool DYNAMIC_ALLOCATION=true>
        using State = dense::State;
    };
    template<typename SPEC>
    struct LayerBackward: public LayerForward<SPEC>{
        // This layer supports backpropagation wrt its input but not its weights (for this it stores the intermediate pre_activations)

        using PARENT = LayerForward<SPEC>;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Activation>;
        using PRE_ACTIVATIONS_CONTAINER_SPEC = matrix::Specification<T, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
        using PRE_ACTIVATIONS_CONTAINER_TYPE = Matrix<PRE_ACTIVATIONS_CONTAINER_SPEC>;
        PRE_ACTIVATIONS_CONTAINER_TYPE pre_activations;
    };
    template<typename SPEC>
    struct LayerGradient: public LayerBackward<SPEC>{
        // This layer supports backpropagation wrt its input but including its weights (for this it stores the intermediate outputs in addition to the pre_activations because they determine the gradient wrt the weights of the following layer)

        using PARENT = LayerBackward<SPEC>;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Activation>;
        using OUTPUT_CONTAINER_SPEC = matrix::Specification<T, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
        using OUTPUT_CONTAINER_TYPE = Matrix<OUTPUT_CONTAINER_SPEC>;
        OUTPUT_CONTAINER_TYPE output;
    };
    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using Layer =
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward,
            LayerForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward,
            LayerBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient,
            LayerGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

    template <typename CONFIG>
    struct BindConfiguration{
        template <typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = nn::layers::dense::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif