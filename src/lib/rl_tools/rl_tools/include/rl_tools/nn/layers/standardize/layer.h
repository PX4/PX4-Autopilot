#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_STANDARDIZE_LAYER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_STANDARDIZE_LAYER_H
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"

#include "../../../nn/nn.h"
#include "../../../nn/parameters/parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::standardize {
    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    constexpr bool check_input_output_f(){
        static_assert(INPUT_SPEC::COLS == LAYER_SPEC::INPUT_DIM);
        static_assert(OUTPUT_SPEC::COLS == LAYER_SPEC::OUTPUT_DIM);
        static_assert(LAYER_SPEC::INPUT_DIM == LAYER_SPEC::OUTPUT_DIM);
        static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
        //                INPUT_SPEC::ROWS <= OUTPUT_SPEC::ROWS && // todo: could be relaxed to not fill the full output
        // static_assert(utils::typing::is_same_v<typename LAYER_SPEC::T, typename INPUT_SPEC::T>);
        // static_assert(utils::typing::is_same_v<typename INPUT_SPEC::T, typename OUTPUT_SPEC::T>);
        return true;
    }
    template <typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    constexpr bool check_input_output = check_input_output_f<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>();

    template <typename LAYER_SPEC_1, typename LAYER_SPEC_2>
    constexpr bool check_compatibility_f(){
        static_assert(LAYER_SPEC_1::INPUT_DIM == LAYER_SPEC_2::INPUT_DIM);
        static_assert(LAYER_SPEC_1::OUTPUT_DIM == LAYER_SPEC_2::OUTPUT_DIM);
        static_assert(LAYER_SPEC_1::OUTPUT_DIM == LAYER_SPEC_1::OUTPUT_DIM);
        return true;
    }

    template <typename LAYER_SPEC_1, typename LAYER_SPEC_2>
    constexpr bool check_compatibility = check_compatibility_f<LAYER_SPEC_1, LAYER_SPEC_2>();
    template<typename T_TYPE_POLICY, typename T_TI>
    struct Configuration {
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        // Summary
        static constexpr auto NUM_WEIGHTS = 0; //zero trainable parameters (the point is to not learn the mean and std by gradient descent, otherwise it would just be a normal layer)
    };
    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using CONFIG = T_CONFIG;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        using CAPABILITY = T_CAPABILITY;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        static constexpr TI OUTPUT_DIM = INPUT_DIM;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = tensor::Replace<NEW_INPUT_SHAPE, OUTPUT_DIM, length(NEW_INPUT_SHAPE{})-1>;
        using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
        static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<INPUT_SHAPE>>{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
        static constexpr TI NUM_WEIGHTS = OUTPUT_DIM * INPUT_DIM + OUTPUT_DIM;
    };
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
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>;
        using STATISTICS_SHAPE = tensor::Shape<TI, INPUT_DIM>;
        using STATISTICS_PARAMETER_SPEC = nn::parameters::Plain::Specification<typename SPEC::TYPE_POLICY, TI, STATISTICS_SHAPE, nn::parameters::groups::Normal, nn::parameters::categories::Constant, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>; // Constant from the view of a forward or backward pass
        typename nn::parameters::Plain::template Instance<STATISTICS_PARAMETER_SPEC> mean, precision; // precision = 1/std

        template<bool DYNAMIC_ALLOCATION=true>
        using State = standardize::State;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = standardize::Buffer;
    };
    template<typename SPEC>
    struct LayerBackward: public LayerForward<SPEC> {
    };
    template<typename SPEC>
    struct LayerGradient: public LayerBackward<SPEC> {
        // This layer supports backpropagation wrt its input but including its weights (for this it stores the intermediate outputs in addition to the pre_activations because they determine the gradient wrt the weights of the following layer)
        using OUTPUT_CONTAINER_SPEC = matrix::Specification<typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM, SPEC::DYNAMIC_ALLOCATION, matrix::layouts::DEFAULT<typename SPEC::TI>, SPEC::CONST>;
        using OUTPUT_CONTAINER_TYPE = Matrix<OUTPUT_CONTAINER_SPEC>;
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
        using Layer = nn::layers::standardize::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
