#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_LAYER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_LAYER_H
#include "../../../nn/activation_functions.h"
#include "../../../nn/parameters/parameters.h"
#include "../../../nn/capability/capability.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::layers::gru{
    template<typename T_TYPE_POLICY, typename T_TI, T_TI T_HIDDEN_DIM, typename T_PARAMETER_GROUP=parameters::groups::Normal, bool T_FAST_TANH = false>
    struct Configuration{
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI = T_TI;
        static constexpr T_TI HIDDEN_DIM = T_HIDDEN_DIM;
        using PARAMETER_GROUP = T_PARAMETER_GROUP;
        static constexpr bool FAST_TANH = T_FAST_TANH;
        static constexpr bool LEARN_INITIAL_HIDDEN_STATE = false;
        // Summary
//        static constexpr auto NUM_WEIGHTS = HIDDEN_DIM * INPUT_DIM + HIDDEN_DIM; // todo
    };
    template <typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CAPABILITY, T_CONFIG{
        using CONFIG = T_CONFIG;
        using CAPABILITY = T_CAPABILITY;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        static_assert(length(INPUT_SHAPE{}) == 3, "The input shape of the GRU must be 3 dimensional for now (sequence x batch x features)");
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        static constexpr TI SEQUENCE_LENGTH = get<length(INPUT_SHAPE{})-3>(INPUT_SHAPE{});
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = tensor::Replace<NEW_INPUT_SHAPE, CONFIG::HIDDEN_DIM, length(NEW_INPUT_SHAPE{})-1>;
        using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
        static constexpr TI INTERNAL_BATCH_SIZE = get<1>(INPUT_SHAPE{}); // Since the Dense layer is based on Matrices (2D Tensors) the dense layer operation is broadcasted over the leading dimensions. Hence, the actual batch size is the product of all leading dimensions, excluding the last one (containing the features). Since rl_tools::matrix_view is used for zero-cost conversion the INTERNAL_BATCH_SIZE accounts for all leading dimensions.
        static constexpr TI NUM_WEIGHTS = CONFIG::HIDDEN_DIM * (INPUT_DIM + 1) * 3 + CONFIG::HIDDEN_DIM * (CONFIG::HIDDEN_DIM+1) * 3;
    };

    namespace buffers{
        template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
        struct Specification{
            using SPEC = T_SPEC;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
        };
        template <typename T_SPEC>
        struct Evaluation{
            using BUFFER_SPEC = T_SPEC;
            using GRU_SPEC = typename BUFFER_SPEC::SPEC;
            using TYPE_POLICY = typename GRU_SPEC::TYPE_POLICY;
            using TI = typename GRU_SPEC::TI;
            static constexpr TI BATCH_SIZE = GRU_SPEC::INTERNAL_BATCH_SIZE;
            using T_ACC = typename TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
            using POST_ACTIVATION_SPEC = tensor::Specification<T_ACC, TI, tensor::Shape<TI, BATCH_SIZE, 3*GRU_SPEC::HIDDEN_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            Tensor<POST_ACTIVATION_SPEC> post_activation;
            using N_PRE_PRE_ACTIVATION_SPEC = tensor::Specification<T_ACC, TI, tensor::Shape<TI, BATCH_SIZE, GRU_SPEC::HIDDEN_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            Tensor<N_PRE_PRE_ACTIVATION_SPEC> n_pre_pre_activation;
            using STEP_BY_STEP_OUTPUT_SPEC = tensor::Specification<T_ACC, TI, tensor::Shape<TI, 1, BATCH_SIZE, GRU_SPEC::HIDDEN_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            Tensor<STEP_BY_STEP_OUTPUT_SPEC> step_by_step_output;

            using PREVIOUS_OUTPUT_SCRATCH_SPEC = tensor::Specification<T_ACC, TI, tensor::Shape<TI, BATCH_SIZE, GRU_SPEC::HIDDEN_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            Tensor<PREVIOUS_OUTPUT_SCRATCH_SPEC> previous_output_scratch;
        };
    }

    template <typename T_TI, typename T_RESET_CONTAINER_TYPE>
    struct ResetModeSpecification{
        using TI = T_TI;
        using RESET_CONTAINER_TYPE = T_RESET_CONTAINER_TYPE;
    };
    template <typename T_BASE, typename T_SPEC>
    struct ResetMode: T_BASE{
        using SPEC = T_SPEC;
        using BASE = T_BASE;
        using PRE = typename SPEC::RESET_CONTAINER_TYPE;
        using RESET_CONTAINER_TYPE = utils::typing::conditional_t<utils::typing::is_reference_v<PRE>, utils::typing::remove_reference_t<PRE>, PRE>;
        RESET_CONTAINER_TYPE reset_container;
    };

    template <typename T_BASE, typename T_SPEC = bool>
    struct NoAutoResetMode: T_BASE{
        using SPEC = T_SPEC;
        using BASE = T_BASE;
    };

    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
    struct StateSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };
    template <typename T_SPEC>
    struct State{
        using STATE_SPEC = T_SPEC;
        using SPEC = typename STATE_SPEC::SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;

        using T_ACC = typename TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;

        using STATE_SHAPE = tensor::Shape<TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::HIDDEN_DIM>;
        using STATE_CONTAINER_SPEC = tensor::Specification<T_ACC, TI, STATE_SHAPE, STATE_SPEC::DYNAMIC_ALLOCATION>;
        using STATE_CONTAINER_TYPE = Tensor<STATE_CONTAINER_SPEC>;
        STATE_CONTAINER_TYPE state;
        using STEP_SHAPE = tensor::Shape<TI, SPEC::INTERNAL_BATCH_SIZE>;
        using STEP_CONTAINER_SPEC = tensor::Specification<TI, TI, STEP_SHAPE, STATE_SPEC::DYNAMIC_ALLOCATION>;
        using STEP_CONTAINER_TYPE = Tensor<STEP_CONTAINER_SPEC>;
        STEP_CONTAINER_TYPE step;
    };

    template<typename T_SPEC>
    struct LayerForward{
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        static constexpr TI SEQUENCE_LENGTH = SPEC::SEQUENCE_LENGTH;
        static constexpr TI INPUT_DIM = SPEC::INPUT_DIM;
        static constexpr TI HIDDEN_DIM = SPEC::HIDDEN_DIM;
        static constexpr TI OUTPUT_DIM = SPEC::HIDDEN_DIM;
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        template <typename NEW_INPUT_SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<NEW_INPUT_SHAPE>;
        using WEIGHTS_INPUT_SHAPE = tensor::Shape<TI, 3*HIDDEN_DIM, INPUT_DIM>;
        using WEIGHTS_INPUT_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, WEIGHTS_INPUT_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Weights, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<WEIGHTS_INPUT_PARAMETER_SPEC> weights_input;

        using BIASES_INPUT_SHAPE = tensor::Shape<TI, 3*HIDDEN_DIM>;
        using BIASES_INPUT_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, BIASES_INPUT_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Biases, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<BIASES_INPUT_PARAMETER_SPEC> biases_input;

        using WEIGHTS_HIDDEN_SHAPE = tensor::Shape<TI, 3*HIDDEN_DIM, HIDDEN_DIM>;
        using WEIGHTS_HIDDEN_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, WEIGHTS_HIDDEN_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Weights, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<WEIGHTS_HIDDEN_PARAMETER_SPEC> weights_hidden;

        using BIASES_HIDDEN_SHAPE = tensor::Shape<TI, 3*HIDDEN_DIM>;
        using BIASES_HIDDEN_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, BIASES_HIDDEN_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Biases, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<BIASES_HIDDEN_PARAMETER_SPEC> biases_hidden;

        using INITIAL_HIDDEN_STATE_SHAPE = tensor::Shape<TI, HIDDEN_DIM>;
        using INITIAL_HIDDEN_STATE_PARAMETER_SPEC = typename SPEC::PARAMETER_TYPE::template Specification<TYPE_POLICY, TI, INITIAL_HIDDEN_STATE_SHAPE, typename SPEC::PARAMETER_GROUP, nn::parameters::categories::Biases, SPEC::DYNAMIC_ALLOCATION, SPEC::CONST>;
        typename SPEC::PARAMETER_TYPE::template Instance<INITIAL_HIDDEN_STATE_PARAMETER_SPEC> initial_hidden_state;

        template<bool DYNAMIC_ALLOCATION=true>
        using State = State<StateSpecification<SPEC, DYNAMIC_ALLOCATION>>;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = buffers::Evaluation<buffers::Specification<SPEC, DYNAMIC_ALLOCATION>>;
    };

    namespace buffers{
        template <typename T_SPEC>
        struct Backward: Evaluation<T_SPEC>{
            using BUFFER_SPEC = T_SPEC;
            using LAYER_SPEC = typename T_SPEC::SPEC;
            using TYPE_POLICY = typename LAYER_SPEC::TYPE_POLICY;
            using TI = typename LAYER_SPEC::TI;
            static constexpr TI INTERNAL_BATCH_SIZE = LAYER_SPEC::INTERNAL_BATCH_SIZE;
            using T_ACC = typename TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
            using TENSOR_SPEC = tensor::Specification<T_ACC, TI, tensor::Shape<TI, INTERNAL_BATCH_SIZE, 3*LAYER_SPEC::HIDDEN_DIM>, BUFFER_SPEC::DYNAMIC_ALLOCATION>;
            using BUFFER_TYPE = Tensor<TENSOR_SPEC>;
            BUFFER_TYPE buffer, buffer2;
            typename decltype(buffer)::template VIEW_RANGE<tensor::ViewSpec<1, 2*LAYER_SPEC::HIDDEN_DIM>> buffer_rz;
            typename decltype(buffer)::template VIEW_RANGE<tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>> buffer_r, buffer_z, buffer_n;
        };
    }

    template<typename T_SPEC>
    struct LayerBackward: LayerForward<T_SPEC>{
        using PARENT = LayerForward<T_SPEC>;
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        using T_ACC = typename TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        using FULL_HIDDEN_SHAPE = tensor::Shape<TI, SPEC::SEQUENCE_LENGTH, SPEC::INTERNAL_BATCH_SIZE, 3*SPEC::HIDDEN_DIM>;
        using FULL_HIDDEN_SPEC = tensor::Specification<T_ACC, TI, FULL_HIDDEN_SHAPE, SPEC::DYNAMIC_ALLOCATION, tensor::RowMajorStride<FULL_HIDDEN_SHAPE>, SPEC::CONST>;
        using FULL_HIDDEN_TYPE = Tensor<FULL_HIDDEN_SPEC>;
        FULL_HIDDEN_TYPE post_activation;
        using SINGLE_HIDDEN_SHAPE = tensor::Shape<TI, SPEC::SEQUENCE_LENGTH, SPEC::INTERNAL_BATCH_SIZE, SPEC::HIDDEN_DIM>;
        using HIDDEN_SPEC = tensor::Specification<T_ACC, TI, SINGLE_HIDDEN_SHAPE, SPEC::DYNAMIC_ALLOCATION, tensor::RowMajorStride<SINGLE_HIDDEN_SHAPE>, SPEC::CONST>;
        using HIDDEN_TYPE = Tensor<HIDDEN_SPEC>;
        HIDDEN_TYPE n_pre_pre_activation;
        using OUTPUT_SPEC = tensor::Specification<T_ACC, TI, SINGLE_HIDDEN_SHAPE, SPEC::DYNAMIC_ALLOCATION, tensor::RowMajorStride<SINGLE_HIDDEN_SHAPE>, SPEC::CONST>;
        using OUTPUT_TYPE = Tensor<OUTPUT_SPEC>;
        OUTPUT_TYPE output;
    };
    template<typename T_SPEC>
    struct LayerGradient: LayerBackward<T_SPEC>{
        using PARENT = LayerBackward<T_SPEC>;
        using TI = typename T_SPEC::TI;
        template<bool DYNAMIC_ALLOCATION=true>
        using Buffer = buffers::Backward<buffers::Specification<T_SPEC, DYNAMIC_ALLOCATION>>;
    };

    template <typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    struct assert_input_output{
        static_assert(length(typename INPUT_SPEC::SHAPE{}) == 3 && length(typename OUTPUT_SPEC::SHAPE{}) == 3);
        static_assert(get<0>(typename INPUT_SPEC::SHAPE{}) == get<0>(typename OUTPUT_SPEC::SHAPE{}));
        static_assert(get<1>(typename INPUT_SPEC::SHAPE{}) == get<1>(typename OUTPUT_SPEC::SHAPE{}));
        static_assert(get<2>(typename INPUT_SPEC::SHAPE{}) == SPEC::INPUT_DIM && get<2>(typename OUTPUT_SPEC::SHAPE{}) == SPEC::HIDDEN_DIM);
    };
    template <typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    bool constexpr check_input_output = length(typename INPUT_SPEC::SHAPE{}) == 3 && length(typename OUTPUT_SPEC::SHAPE{}) == 3 &&
            get<0>(typename INPUT_SPEC::SHAPE{}) <= get<0>(typename OUTPUT_SPEC::SHAPE{}) &&
            get<1>(typename INPUT_SPEC::SHAPE{}) <= get<1>(typename OUTPUT_SPEC::SHAPE{}) &&
            get<2>(typename INPUT_SPEC::SHAPE{}) == SPEC::INPUT_DIM && get<2>(typename OUTPUT_SPEC::SHAPE{}) == SPEC::HIDDEN_DIM;

    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using Layer =
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward, LayerForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward, LayerBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
    typename utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Gradient, LayerGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>, void>>>;

    template <typename T_CONFIG>
    struct BindConfiguration{
        template <typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = nn::layers::gru::Layer<T_CONFIG, CAPABILITY, INPUT_SHAPE>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif