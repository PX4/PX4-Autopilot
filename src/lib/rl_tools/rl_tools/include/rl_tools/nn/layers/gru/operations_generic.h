#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_OPERATIONS_GENERIC_H

#include "layer.h"
#include "helper_operations_generic.h"
#include "../../parameters/operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::gru::LayerForward<SPEC>& layer){
        malloc(device, layer.weights_input);
        malloc(device, layer.biases_input);
        malloc(device, layer.weights_hidden);
        malloc(device, layer.biases_hidden);

        malloc(device, layer.initial_hidden_state);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::gru::LayerBackward<SPEC>& layer){
        malloc(device, static_cast<nn::layers::gru::LayerForward<SPEC>&>(layer));
        malloc(device, layer.n_pre_pre_activation);
        malloc(device, layer.post_activation);
        malloc(device, layer.output);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::gru::buffers::Evaluation<SPEC>& buffers){
        malloc(device, buffers.post_activation);
        malloc(device, buffers.n_pre_pre_activation);
        malloc(device, buffers.step_by_step_output);
        malloc(device, buffers.previous_output_scratch);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::gru::buffers::Evaluation<SPEC>& buffers){
        free(device, buffers.post_activation);
        free(device, buffers.n_pre_pre_activation);
        free(device, buffers.step_by_step_output);
        free(device, buffers.previous_output_scratch);
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void init(DEVICE& device, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers){
        using SPEC = typename BUFFER_SPEC::SPEC;
        using VIEW_SPEC_DOUBLE = tensor::ViewSpec<1, 2*SPEC::HIDDEN_DIM>;
        buffers.buffer_rz = view_range(device, buffers.buffer, 0*SPEC::HIDDEN_DIM, VIEW_SPEC_DOUBLE{});
        using VIEW_SPEC = tensor::ViewSpec<1, SPEC::HIDDEN_DIM>;
        buffers.buffer_r = view_range(device, buffers.buffer, 0*SPEC::HIDDEN_DIM, VIEW_SPEC{});
        buffers.buffer_z = view_range(device, buffers.buffer, 1*SPEC::HIDDEN_DIM, VIEW_SPEC{});
        buffers.buffer_n = view_range(device, buffers.buffer, 2*SPEC::HIDDEN_DIM, VIEW_SPEC{});
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::gru::buffers::Backward<SPEC>& buffers){
        malloc(device, static_cast<nn::layers::gru::buffers::Evaluation<SPEC>&>(buffers));
        malloc(device, buffers.buffer);
        malloc(device, buffers.buffer2);
        init(device, buffers);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::gru::buffers::Backward<SPEC>& buffer){
        free(device, static_cast<nn::layers::gru::buffers::Evaluation<SPEC>&>(buffer));
        free(device, buffer.buffer);
        free(device, buffer.buffer2);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::gru::State<SPEC>& state){
        malloc(device, state.state);
        malloc(device, state.step);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::gru::State<SOURCE_SPEC>& source, nn::layers::gru::State<TARGET_SPEC>& target){
        copy(source_device, target_device, source.state, target.state);
        copy(source_device, target_device, source.step, target.step);
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_truncate(DEVICE& device, const nn::layers::gru::LayerForward<SPEC>& layer, nn::layers::gru::State<STATE_SPEC>& state, Mode<MODE> mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static constexpr TI BATCH_SIZE = get<0>(typename decltype(state.state)::SPEC::SHAPE{});
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            if(!mode::is<MODE, nn::layers::gru::NoAutoResetMode> && get(device, state.step, batch_i) >= SPEC::SEQUENCE_LENGTH){
                auto row = view(device, state.state, batch_i);
                copy(device, device, layer.initial_hidden_state.parameters, row);
                set(device, state.step, 0, batch_i);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename STATE_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::gru::LayerForward<SPEC>& layer, nn::layers::gru::State<STATE_SPEC>& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) {
        using TI = typename DEVICE::index_t;
        static constexpr TI BATCH_SIZE = get<0>(typename decltype(state.state)::SPEC::SHAPE{});
        if constexpr(mode::is<MODE, mode::sequential::ResetMask>){
            for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
                if (get(mode.mask, 0, batch_i)) {
                    set(device, state.step, batch_i, 0);
                    auto row = view(device, state.state, batch_i);
                    copy(device, device, layer.initial_hidden_state.parameters, row);
                }
            }
        }
        else{
            if constexpr(mode::is<MODE, mode::Default>){
                set_all(device, state.step, 0);
                for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
                    auto row = view(device, state.state, batch_i);
                    copy(device, device, layer.initial_hidden_state.parameters, row);
                }
            }
            else{
                utils::assert_exit(device, false, "Unsupported mode");
            }
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::gru::State<SPEC>& state){
        free(device, state.state);
        free(device, state.step);
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::gru::LayerForward<SPEC>& l, RNG& rng){
        // same as in PyTorch
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        T scale_input = 1 / math::sqrt(device.math, (T)SPEC::INPUT_DIM);
        T scale_hidden = 1 / math::sqrt(device.math, (T)SPEC::HIDDEN_DIM);
        rand(device, l.weights_hidden.parameters, rng, -scale_hidden, scale_hidden);
        rand(device, l.weights_input.parameters, rng, -scale_input, scale_input);
        rand(device, l.biases_hidden.parameters, rng, -scale_hidden, scale_hidden);
        rand(device, l.biases_input.parameters, rng, -scale_input, scale_input);
        set_all(device, l.initial_hidden_state.parameters, 0);
    }
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_OUTPUT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_broadcast_accumulate(DEVICE& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, Tensor<SPEC_OUTPUT>& t_output){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::multiply_broadcast_accumulate");
#endif
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 1);
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_OUTPUT::SHAPE{}));
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_OUTPUT::SHAPE{}) == get<1>(typename SPEC_1::SHAPE{}));
        using TI = typename DEVICE::index_t;
        using T = typename SPEC_1::T;
        // for(TI i=0; i < get<0>(typename SPEC_1::SHAPE{}); i++){
        //     for(TI j=0; j < get<1>(typename SPEC_1::SHAPE{}); j++){
        //         T t1_value = get(device, t1, i, j);
        //         T t2_value = get(device, t2, j);
        //         T t_output_value = get(device, t_output, i, j);
        //         set(device, t_output, t1_value * t2_value + t_output_value, i, j);
        //     }
        // }
        using NEW_SHAPE = tensor::Shape<TI, SPEC_1::SHAPE::template GET<0>, SPEC_2::SHAPE::template GET<0>>;;
        using NEW_STRIDE = tensor::Stride<TI, 0, SPEC_2::STRIDE::template GET<0>>;
        using NEW_SPEC = tensor::Specification<typename SPEC_2::T, typename SPEC_2::TI, NEW_SHAPE, true, NEW_STRIDE, true>;
        const Tensor<NEW_SPEC> t2_broadcasted{{data(t2)}};
        multiply_accumulate(device, t1, t2_broadcasted, t_output);
    }

    namespace nn::layers::gru{
        namespace multi_step_intermediates_tag_dispatch{ // this is required for MSVC because it complains about the slicker SFINAE-based version
            struct three_dimensional_tag{};
            struct two_dimensional_tag{};

            template <typename SPEC>
            struct dimension_tag{
                using type = typename utils::typing::conditional_t<length(typename SPEC::SHAPE{})==3, three_dimensional_tag, two_dimensional_tag>;
            };
            template <auto BATCH_SIZE, typename DEVICE, typename SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT auto impl(DEVICE& device, Tensor<SPEC>& tensor, typename DEVICE::index_t step_i, three_dimensional_tag){
                auto post_activation = view_range(device, tensor, 0, tensor::ViewSpec<1, BATCH_SIZE>{});
                return view(device, post_activation, step_i);
            }
            template <auto BATCH_SIZE, typename DEVICE, typename SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT auto impl(DEVICE& device, Tensor<SPEC>& tensor, typename DEVICE::index_t step_i, two_dimensional_tag){
                auto post_activation = view_range(device, tensor, 0, tensor::ViewSpec<0, BATCH_SIZE>{});
                return post_activation;
            }
        }
        namespace mode{

            template <typename SPEC, typename MODE, typename MODE_SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT bool reset_full_batch(const Mode<nn::layers::gru::ResetMode<MODE, MODE_SPEC>>& mode){
                return false;
            }
            template <typename SPEC, typename MODE>
            RL_TOOLS_FUNCTION_PLACEMENT bool reset_full_batch(const Mode<MODE>& mode){
                return false;
            }

            template <typename SPEC, typename MODE, typename MODE_SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT constexpr bool can_reset_sample(const Mode<nn::layers::gru::ResetMode<MODE, MODE_SPEC>>){
                return true;
            }
            template <typename SPEC, typename MODE>
            RL_TOOLS_FUNCTION_PLACEMENT constexpr bool can_reset_sample(const Mode<MODE>){
                return false;
            }

            template <typename SPEC, typename DEVICE, typename MODE, typename MODE_SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT bool reset_sample(DEVICE& device, const Mode<nn::layers::gru::ResetMode<MODE, MODE_SPEC>>& mode, typename MODE_SPEC::TI step_i, typename MODE_SPEC::TI batch_sample_i){
                return get(device, mode.reset_container, step_i, batch_sample_i, (typename MODE_SPEC::TI)0);
            }
            template <typename SPEC, typename DEVICE, typename MODE, typename TI>
            RL_TOOLS_FUNCTION_PLACEMENT bool reset_sample(DEVICE& device, const Mode<MODE>& mode, TI step_i, TI batch_sample_i){
                return false;
            }

            template <typename DEVICE, typename MODE, typename MODE_SPEC>
            RL_TOOLS_FUNCTION_PLACEMENT typename DEVICE::index_t num_resets(DEVICE& device, const Mode<nn::layers::gru::ResetMode<MODE, MODE_SPEC>>& mode){
                return cast_reduce_sum<typename MODE_SPEC::TI>(device, mode.reset_container);
            }
            template <typename DEVICE, typename MODE, bool SHOULD_NEVER=false>
            RL_TOOLS_FUNCTION_PLACEMENT typename DEVICE::index_t num_resets(DEVICE& device, const Mode<MODE>& mode){
                static_assert(SHOULD_NEVER, "This should not be called");
                return 1;
            }
        }

        template <auto BATCH_SIZE, typename DEVICE, typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT auto multi_step_intermediates(DEVICE& device, Tensor<SPEC>& tensor, typename DEVICE::index_t step_i) {
            return multi_step_intermediates_tag_dispatch::impl<BATCH_SIZE>(device, tensor, step_i, typename multi_step_intermediates_tag_dispatch::dimension_tag<SPEC>::type{});
        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename POST_ACTIVATION_SPEC, typename N_PRE_PRE_ACTIVATION_SPEC, typename PREVIOUS_OUTPUT_SCRATCH, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::gru::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<POST_ACTIVATION_SPEC>& post_activation_full, Tensor<N_PRE_PRE_ACTIVATION_SPEC>& n_pre_pre_activation_full, Tensor<OUTPUT_SPEC>& output_full, Tensor<PREVIOUS_OUTPUT_SCRATCH>& previous_output_scratch_full, nn::layers::gru::buffers::Evaluation<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        constexpr TI SEQUENCE_LENGTH = get<0>(typename INPUT_SPEC::SHAPE{});
        constexpr TI BATCH_SIZE = get<1>(typename INPUT_SPEC::SHAPE{});
        // you can either pass post_activation and n_pre_pre_activation as intermediate tensors with steps (e.g. for "forward" to maintain the state for the backward pass) or in a purely iterative fashion
        static_assert(nn::layers::gru::check_input_output<LAYER_SPEC, INPUT_SPEC, OUTPUT_SPEC>, "Input and output spec not matching");
        static_assert(length(typename POST_ACTIVATION_SPEC::SHAPE{}) == 3 || length(typename POST_ACTIVATION_SPEC::SHAPE{}) == 2);
        static_assert(length(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{}) == 3 || length(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{}) == 2);
        constexpr bool MULTI_STEP_INTERMEDIATES = length(typename POST_ACTIVATION_SPEC::SHAPE{}) == 3;
        constexpr TI BUFFER_BATCH_SIZE = get<length(typename POST_ACTIVATION_SPEC::SHAPE{})-2>(typename POST_ACTIVATION_SPEC::SHAPE{});
        static_assert(BATCH_SIZE <= BUFFER_BATCH_SIZE);
        static_assert(MULTI_STEP_INTERMEDIATES == (length(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{}) == 3), "N_PRE_PRE_ACTIVATION_SPEC must have the same number of dimensions as POST_ACTIVATION_SPEC");
        static_assert(get<length(typename POST_ACTIVATION_SPEC::SHAPE{})-1>(typename POST_ACTIVATION_SPEC::SHAPE{}) == 3*LAYER_SPEC::HIDDEN_DIM);
        static_assert(get<length(typename POST_ACTIVATION_SPEC::SHAPE{})-2>(typename POST_ACTIVATION_SPEC::SHAPE{}) >= BATCH_SIZE);
        static_assert(get<length(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{})-1>(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{}) == LAYER_SPEC::HIDDEN_DIM);
        static_assert(get<length(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{})-2>(typename N_PRE_PRE_ACTIVATION_SPEC::SHAPE{}) >= BATCH_SIZE);

        auto previous_output_scratch = view_range(device, previous_output_scratch_full, 0, tensor::ViewSpec<0, BATCH_SIZE>{});
        auto output = view_range(device, output_full, 0, tensor::ViewSpec<1, BATCH_SIZE>{});


//        constexpr bool STEP_BY_STEP = mode::is<MODE, nn::layers::gru::StepByStepMode>;
        constexpr bool CAN_RESET_SAMPLE = nn::layers::gru::mode::can_reset_sample<LAYER_SPEC>(utils::typing::remove_reference_t<decltype(mode)>{});
        for(TI step_i=0; step_i < SEQUENCE_LENGTH; step_i++){
            bool reset_full_batch = nn::layers::gru::mode::reset_full_batch<LAYER_SPEC>(mode) || (step_i == 0);
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::evaluate_step");
#endif
            auto input_step = view(device, input, step_i);
            auto post_activation_step = nn::layers::gru::multi_step_intermediates<BATCH_SIZE>(device, post_activation_full, step_i);
            auto n_pre_pre_activation_step = nn::layers::gru::multi_step_intermediates<BATCH_SIZE>(device, n_pre_pre_activation_full, step_i);
            auto output_step = view(device, output, step_i);

            auto rz_post_activation = view_range(device, post_activation_step, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, 2*LAYER_SPEC::HIDDEN_DIM>{});
            auto r_post_activation  = view_range(device, post_activation_step, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
            auto z_post_activation  = view_range(device, post_activation_step, 1*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
            auto n_post_activation  = view_range(device, post_activation_step, 2*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});

            if(reset_full_batch){
                nn::layers::gru::helper::matrix_multiply_broadcast_transpose_bias(device, layer.weights_hidden.parameters, layer.initial_hidden_state.parameters, layer.biases_hidden.parameters, post_activation_step);
            }
            else{
                if constexpr(CAN_RESET_SAMPLE){
                    #ifdef RL_TOOLS_ENABLE_TRACY
                    ZoneScopedN("gru::evaluate_step::CAN_RESET_SAMPLE_1");
                    #endif
                    for(TI sample_i = 0; sample_i < BATCH_SIZE; sample_i++){
                        auto target = view(device, previous_output_scratch, sample_i);
                        bool reset = nn::layers::gru::mode::reset_sample<LAYER_SPEC>(device, mode, step_i, sample_i);
                        if(reset){
                            auto source = layer.initial_hidden_state.parameters;
                            copy(device, device, source, target);
                        }
                        else{
                            auto output_previous_step = view(device, output, step_i-1);
                            auto source = view(device, output_previous_step, sample_i);
                            copy(device, device, source, target);
                        }
                    }
                    nn::layers::gru::helper::matrix_multiply_transpose_bias(device, layer.weights_hidden.parameters, previous_output_scratch, layer.biases_hidden.parameters, post_activation_step);
                }
                else{
                    auto output_previous_step = view(device, output, step_i-1);
                    nn::layers::gru::helper::matrix_multiply_transpose_bias(device, layer.weights_hidden.parameters, output_previous_step, layer.biases_hidden.parameters, post_activation_step);
                }
            }

            copy(device, device, n_post_activation, n_pre_pre_activation_step);
            set_all(device, n_post_activation, 0);

            nn::layers::gru::helper::matrix_multiply_transpose_bias_accumulate(device, layer.weights_input.parameters, input_step, layer.biases_input.parameters, post_activation_step);
            if(LAYER_SPEC::FAST_TANH){
                fast_sigmoid(device, rz_post_activation);
            }
            else{
#ifdef RL_TOOLS_ENABLE_TRACY
                ZoneScopedN("gru::evaluate_step::sigmoid");
#endif
                sigmoid(device, rz_post_activation);
            }
            multiply_accumulate(device, n_pre_pre_activation_step, r_post_activation, n_post_activation);
            if constexpr(LAYER_SPEC::FAST_TANH){
                fast_tanh(device, n_post_activation);
            }
            else{
#ifdef RL_TOOLS_ENABLE_TRACY
                ZoneScopedN("gru::evaluate_step::tanh");
#endif
                tanh(device, n_post_activation);
            }
            one_minus(device, z_post_activation, output_step);
            multiply(device, n_post_activation, output_step);
            if(reset_full_batch){
                multiply_broadcast_accumulate(device, z_post_activation, layer.initial_hidden_state.parameters, output_step);
            }
            else{
                if constexpr(CAN_RESET_SAMPLE) {
#ifdef RL_TOOLS_ENABLE_TRACY
                    ZoneScopedN("gru::evaluate_step::CAN_RESET_SAMPLE_2");
#endif
                    // we don't need to assemble previous_output_scratch again because we know that we assembled it before already
                    multiply_accumulate(device, z_post_activation, previous_output_scratch, output_step);
                }
                else{
                    auto output_previous_step = view(device, output, step_i-1);
                    multiply_accumulate(device, z_post_activation, output_previous_step, output_step);
                }
            }
        }
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::gru::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::gru::buffers::Evaluation<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = get<1>(typename INPUT_SPEC::SHAPE{});
        evaluate(device, layer, input, buffers.post_activation, buffers.n_pre_pre_activation, output, buffers.previous_output_scratch, buffers, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename STATE_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::gru::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, typename nn::layers::gru::State<STATE_SPEC>& state, Tensor<OUTPUT_SPEC>& output, nn::layers::gru::buffers::Evaluation<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(length(typename INPUT_SPEC::SHAPE{}) == 2, "evaluate_step does not have a squence dimension (only batch_size x input_dim)");
        static_assert(length(typename OUTPUT_SPEC::SHAPE{}) == 2);
        constexpr TI BATCH_SIZE = get<0>(typename INPUT_SPEC::SHAPE{});
        auto previous_output_scratch = view_range(device, buffers.previous_output_scratch, 0, tensor::ViewSpec<0, BATCH_SIZE>{});
        auto relevant_state = view_range(device, state.state, 0, tensor::ViewSpec<0, BATCH_SIZE>{});
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::evaluate_step");
#endif
        auto input_step = input;
        auto output_step = output;

        reset_truncate(device, layer, state, mode);

        auto post_activation_batch = view_range(device, buffers.post_activation, 0, tensor::ViewSpec<0, BATCH_SIZE>{});
        auto n_pre_pre_activation_batch = view_range(device, buffers.n_pre_pre_activation, 0, tensor::ViewSpec<0, BATCH_SIZE>{});

        auto rz_post_activation = view_range(device, post_activation_batch, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, 2*LAYER_SPEC::HIDDEN_DIM>{});
        auto r_post_activation  = view_range(device, post_activation_batch, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
        auto z_post_activation  = view_range(device, post_activation_batch, 1*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
        auto n_post_activation  = view_range(device, post_activation_batch, 2*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});

        copy(device, device, relevant_state, previous_output_scratch);
        nn::layers::gru::helper::matrix_multiply_transpose_bias(device, layer.weights_hidden.parameters, relevant_state, layer.biases_hidden.parameters, post_activation_batch);

        copy(device, device, n_post_activation, n_pre_pre_activation_batch);
        set_all(device, n_post_activation, 0);

        nn::layers::gru::helper::matrix_multiply_transpose_bias_accumulate(device, layer.weights_input.parameters, input_step, layer.biases_input.parameters, post_activation_batch);
        if(LAYER_SPEC::FAST_TANH){
            fast_sigmoid(device, rz_post_activation);
        }
        else{
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::evaluate_step::sigmoid");
#endif
            sigmoid(device, rz_post_activation);
        }
        multiply_accumulate(device, n_pre_pre_activation_batch, r_post_activation, n_post_activation);
        if constexpr(LAYER_SPEC::FAST_TANH){
            fast_tanh(device, n_post_activation);
        }
        else{
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::evaluate_step::tanh");
#endif
            tanh(device, n_post_activation);
        }
        one_minus(device, z_post_activation, output_step);
        static_assert(length(typename decltype(z_post_activation)::SPEC::SHAPE{}) == 2);
        static_assert(length(typename decltype(output_step)::SPEC::SHAPE{}) == 2);
        static_assert(get<0>(typename decltype(z_post_activation)::SPEC::SHAPE{}) == get<0>(typename decltype(output_step)::SPEC::SHAPE{}));
        static_assert(get<1>(typename decltype(z_post_activation)::SPEC::SHAPE{}) == get<1>(typename decltype(output_step)::SPEC::SHAPE{}));
        multiply(device, n_post_activation, output_step);
        multiply_accumulate(device, z_post_activation, previous_output_scratch, output_step);
        copy(device, device, output_step, relevant_state);
        for(TI batch_i=0; batch_i < BATCH_SIZE; batch_i++){
            TI new_step = get(device, state.step, batch_i) + 1;
            if(!mode::is<MODE, nn::layers::gru::NoAutoResetMode> && new_step >= LAYER_SPEC::SEQUENCE_LENGTH){
                new_step = 0;
                auto row = view(device, relevant_state, batch_i);
                copy(device, device, layer.initial_hidden_state.parameters, row);
            }
            set(device, state.step, new_step, batch_i);

        }
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename RNG, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::gru::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::gru::buffers::Evaluation<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        evaluate(device, layer, input, layer.post_activation, layer.n_pre_pre_activation, layer.output, buffers.previous_output_scratch, buffers, rng, mode);
    }
    template<typename DEVICE, typename SPEC_FACTOR, typename SPEC_1, typename SPEC_2, typename SPEC_OUTPUT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_subtract_broadcast(DEVICE& device, Tensor<SPEC_FACTOR>& factor, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUTPUT>& t_output) {
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::multiply_subtract_broadcast");
#endif
        // broadcast t1 along first dimension
        static_assert(length(typename SPEC_FACTOR::SHAPE{}) == 2);
        static_assert(get<0>(typename SPEC_FACTOR::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_FACTOR::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{}));
        static_assert(length(typename SPEC_1::SHAPE{}) == 1);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(get<0>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUTPUT::SHAPE{}));
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_OUTPUT::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{}));
        using TI = typename DEVICE::index_t;
        using T = typename SPEC_1::T;
        for(TI i=0; i < get<0>(typename SPEC_2::SHAPE{}); i++){
            for(TI j=0; j < get<1>(typename SPEC_2::SHAPE{}); j++){
                T factor_value = get(device, factor, i, j);
                T t1_value = get(device, t1, j);
                T t2_value = get(device, t2, i, j);
                set(device, t_output, factor_value*(t1_value - t2_value), i, j);
            }
        }
    }
    namespace tensor::operations::ternary{
        template <typename T>
        struct MultiplySubtract: Operation{
            template <typename DEVICE, typename T_T>
            RL_TOOLS_FUNCTION_PLACEMENT T_T static operation(DEVICE& device, const MultiplySubtract<T_T>& parameter, T_T factor, T_T a, T_T b){
                return factor * (a-b);
            }
        };
    }
    template<typename DEVICE, typename SPEC_FACTOR, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_subtract(DEVICE& device, Tensor<SPEC_FACTOR>& factor, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::multiply_subtract");
#endif
        ternary_operation(device, tensor::operations::ternary::MultiplySubtract<typename SPEC_1::T>{}, factor, t1, t2, result);
    }

#ifndef RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    RL_TOOLS_FUNCTION_PLACEMENT void matrix_multiply_broadcast_accumulate(DEVICE& device, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::matrix_multiply_broadcast_accumulate");
#endif
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 1);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 2);
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{}));
        static_assert(get<0>(typename SPEC_2::SHAPE{}) == get<1>(typename SPEC_OUT::SHAPE{}));
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        for(TI row_i=0; row_i < get<0>(typename SPEC_1::SHAPE{}); ++row_i){
            for(TI col_j=0; col_j < get<0>(typename SPEC_2::SHAPE{}); ++col_j){
                T acc = get(device, result, row_i, col_j);
                T t2_value = get(device, t2, col_j);
                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
                    acc += get(device, t1, row_i, k) * t2_value;
                }
                set(device, result, acc, row_i, col_j);
            }
        }
    }
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_OUT>
    RL_TOOLS_FUNCTION_PLACEMENT void matrix_multiply_accumulate_reduce(DEVICE& device, const Tensor<SPEC_1>& t1, const Tensor<SPEC_2>& t2, Tensor<SPEC_OUT>& result){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::matrix_multiply_accumulate_reduce");
#endif
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUT::SHAPE{}) == 1);
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUT::SHAPE{}));
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        for(TI row_i=0; row_i < get<0>(typename SPEC_1::SHAPE{}); ++row_i){
            for(TI col_j=0; col_j < get<1>(typename SPEC_2::SHAPE{}); ++col_j){
                T acc = get(device, result, col_j);
                for(TI k=0; k < get<1>(typename SPEC_1::SHAPE{}); ++k){
                    acc += get(device, t1, row_i, k) * get(device, t2, k, col_j);
                }
                set(device, result, acc, col_j);
            }
        }
    }
#endif
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_OUTPUT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_accumulate_reduce(DEVICE& device, Tensor<SPEC_1>& t1, Tensor<SPEC_2>& t2, Tensor<SPEC_OUTPUT>& t_output){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::multiply_accumulate_reduce");
#endif
        static_assert(length(typename SPEC_1::SHAPE{}) == 2);
        static_assert(length(typename SPEC_2::SHAPE{}) == 2);
        static_assert(length(typename SPEC_OUTPUT::SHAPE{}) == 1);
        static_assert(get<0>(typename SPEC_1::SHAPE{}) == get<0>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_1::SHAPE{}) == get<1>(typename SPEC_2::SHAPE{}));
        static_assert(get<1>(typename SPEC_2::SHAPE{}) == get<0>(typename SPEC_OUTPUT::SHAPE{}));
        using T = typename SPEC_1::T;
        using TI = typename DEVICE::index_t;
        for(TI row_i=0; row_i < get<0>(typename SPEC_1::SHAPE{}); ++row_i){
            for(TI col_j=0; col_j < get<1>(typename SPEC_1::SHAPE{}); ++col_j){
                T increment = get(device, t1, row_i, col_j) * get(device, t2, row_i, col_j);
                set(device, t_output, get(device, t_output, col_j) + increment, col_j);
            }
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer) {
        zero_gradient(device, layer.weights_input);
        zero_gradient(device, layer.biases_input);
        zero_gradient(device, layer.weights_hidden);
        zero_gradient(device, layer.biases_hidden);
        zero_gradient(device, layer.initial_hidden_state);
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        update(device, layer.weights_input, optimizer);
        update(device, layer.biases_input, optimizer);
        update(device, layer.weights_hidden, optimizer);
        update(device, layer.biases_hidden, optimizer);
        update(device, layer.initial_hidden_state, optimizer);
    }

    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        _reset_optimizer_state(device, layer.weights_input, optimizer);
        _reset_optimizer_state(device, layer.biases_input, optimizer);
        _reset_optimizer_state(device, layer.weights_hidden, optimizer);
        _reset_optimizer_state(device, layer.biases_hidden, optimizer);
        _reset_optimizer_state(device, layer.initial_hidden_state, optimizer);
    }

    namespace tensor::operations::ternary{
        // template <typename T>
        // T multiply_one_minus_times_d_tanh_post_activation(T factor, T one_minus, T tanh_post_activation){
        //     return factor * (1-one_minus) * (1-tanh_post_activation*tanh_post_activation);
        // }
        template <typename T>
        struct MultiplyOneMinusTimesDTanhPostActivation: Operation{
            template <typename DEVICE, typename T_T>
            RL_TOOLS_FUNCTION_PLACEMENT T_T static operation(DEVICE& device, const MultiplyOneMinusTimesDTanhPostActivation<T_T>& parameter, T_T factor, T_T one_minus, T_T tanh_post_activation){
                return factor * (1-one_minus) * (1-tanh_post_activation*tanh_post_activation);
            }
        };
    }

    template<typename DEVICE, typename SPEC_FACTOR, typename SPEC_OM, typename SPEC_TANH, typename SPEC_RESULT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_one_minus_times_d_tanh_post_activation(DEVICE& device, Tensor<SPEC_FACTOR>& factor, Tensor<SPEC_OM>& one_minus, Tensor<SPEC_TANH>& tanh_post_activation, Tensor<SPEC_RESULT>& result){
        using T = typename SPEC_FACTOR::T;
        using PARAMETER = T;
        tensor::operations::ternary::MultiplyOneMinusTimesDTanhPostActivation<T> op{};
        ternary_operation(device, op, factor, one_minus, tanh_post_activation, result);
    }
    namespace tensor::operations::binary{
        struct MultiplyDSigmoidPostActivation{
            template <typename DEVICE, typename T_T>
            RL_TOOLS_FUNCTION_PLACEMENT static T_T operation(DEVICE& device, const MultiplyDSigmoidPostActivation& parameters, T_T factor, T_T post_activation){
                return factor * post_activation * (1-post_activation);
            }
        };
    }
    template<typename DEVICE, typename SPEC_FACTOR, typename SPEC_PA, typename SPEC_RESULT>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_d_sigmoid_post_activation(DEVICE& device, Tensor<SPEC_FACTOR>& factor, Tensor<SPEC_PA>& pre_activation, Tensor<SPEC_RESULT>& result){
        using T = typename SPEC_FACTOR::T;
        tensor::operations::binary::MultiplyDSigmoidPostActivation op{};
        binary_operation(device, op, factor, pre_activation, result);
    }

    template<bool CALCULATE_D_INPUT, bool CALCULATE_D_PARAMETERS, typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _backward(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, typename DEVICE::index_t step_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::_backward_step");
#endif
        // call with backward<false> to disable d_input calculation
        // warning: this modifies d_output!
        static_assert(tensor::same_dimensions<typename decltype(layer.output)::SPEC, D_OUTPUT_SPEC>());
        static_assert(tensor::same_dimensions<INPUT_SPEC, D_INPUT_SPEC>());
        static_assert(nn::layers::gru::check_input_output<LAYER_SPEC, INPUT_SPEC, typename decltype(layer.output)::SPEC>, "Input and output spec not matching");
        using TI = typename DEVICE::index_t;
        constexpr TI BATCH_SIZE = get<1>(typename INPUT_SPEC::SHAPE{});
        auto input_step = view(device, input, step_i);
        auto n_pre_pre_activation_step = view(device, layer.n_pre_pre_activation, step_i);
        auto post_activation_step = view(device, layer.post_activation, step_i);
        auto d_output_step = view(device, d_output, step_i);
        auto d_input_step = view(device, d_input, step_i);


        auto rz_post_activation = view_range(device, post_activation_step, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, 2*LAYER_SPEC::HIDDEN_DIM>{});
        auto r_post_activation = view_range(device, post_activation_step, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
        auto z_post_activation = view_range(device, post_activation_step, 1*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});
        auto n_post_activation = view_range(device, post_activation_step, 2*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<1, LAYER_SPEC::HIDDEN_DIM>{});

        constexpr bool CAN_RESET_SAMPLE = nn::layers::gru::mode::can_reset_sample<LAYER_SPEC>(decltype(mode){});
        bool reset_full_batch = nn::layers::gru::mode::reset_full_batch<LAYER_SPEC>(mode) || step_i == 0;
        if(reset_full_batch){
            multiply_subtract_broadcast(device, d_output_step, layer.initial_hidden_state.parameters, n_post_activation, buffers.buffer_z);
            if constexpr (CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
                auto d_output_previous_step = layer.initial_hidden_state.gradient;
                multiply_accumulate_reduce(device, d_output_step, z_post_activation, d_output_previous_step);
            }
        }
        else{
            if constexpr(CAN_RESET_SAMPLE){
                for(TI sample_i = 0; sample_i < BATCH_SIZE; sample_i++){
                    auto target = view(device, buffers.previous_output_scratch, sample_i);
                    auto z_post_activation_sample = view(device, z_post_activation, sample_i);
                    auto d_output_step_sample = view(device, d_output_step, sample_i);
                    bool reset = nn::layers::gru::mode::reset_sample<LAYER_SPEC>(device, mode, step_i, sample_i) || step_i == 0;
                    if(reset){
                        auto source = layer.initial_hidden_state.parameters;
                        copy(device, device, source, target);

                        // also propagate the d_output to the initial state
                        if constexpr(CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
                            auto d_output_previous_step_sample = layer.initial_hidden_state.gradient;
                            multiply_accumulate(device, d_output_step_sample, z_post_activation_sample, d_output_previous_step_sample);
                        }
                    }
                    else{
                        auto output_previous_step = view(device, layer.output, step_i-1);
                        auto source = view(device, output_previous_step, sample_i);
                        copy(device, device, source, target);

                        // also propagate the d_output to the previous step
                        auto d_output_previous_step = view(device, d_output, step_i-1);
                        auto d_output_previous_step_sample = view(device, d_output_previous_step, sample_i);
                        multiply_accumulate(device, d_output_step_sample, z_post_activation_sample, d_output_previous_step_sample);
                    }
                }
                multiply_subtract(device, d_output_step, buffers.previous_output_scratch, n_post_activation, buffers.buffer_z);
            }
            else{
                auto output_previous_step = view(device, layer.output, step_i-1);
                multiply_subtract(device, d_output_step, output_previous_step, n_post_activation, buffers.buffer_z);
                auto d_output_previous_step = view(device, d_output, step_i-1);
                multiply_accumulate(device, d_output_step, z_post_activation, d_output_previous_step);
            }
        }
        multiply_one_minus_times_d_tanh_post_activation(device, d_output_step, z_post_activation, n_post_activation, buffers.buffer_n);
        multiply(device, buffers.buffer_n, n_pre_pre_activation_step, buffers.buffer_r);
        multiply_d_sigmoid_post_activation(device, buffers.buffer_rz, rz_post_activation, buffers.buffer_rz);
        auto buffer_transpose = permute(device, buffers.buffer, tensor::PermutationSpec<1, 0>{});
        static_assert(decltype(buffer_transpose)::SPEC::SIZE == decltype(buffers.buffer)::SPEC::SIZE);
        if constexpr(CALCULATE_D_PARAMETERS){
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::CALCULATE_D_PARAMETERS::weights_input");
#endif
            matrix_multiply_accumulate(device, buffer_transpose, input_step, layer.weights_input.gradient);
            reduce_sum<true>(device, buffer_transpose, layer.biases_input.gradient);
            auto b_irz_grad = view_range(device, layer.biases_input.gradient, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<0, 2*LAYER_SPEC::HIDDEN_DIM>{});
            auto b_hrz_grad = view_range(device, layer.biases_hidden.gradient, 0*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<0, 2*LAYER_SPEC::HIDDEN_DIM>{});
            copy(device, device, b_irz_grad, b_hrz_grad);
        }

        if constexpr(CALCULATE_D_INPUT){
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::CALCULATE_D_INPUT::d_input");
#endif
            matrix_multiply(device, buffers.buffer, layer.weights_input.parameters, d_input_step);
        }

        multiply(device, r_post_activation, buffers.buffer_n);

        if(reset_full_batch){
            if constexpr(CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
                matrix_multiply_broadcast_accumulate(device, buffer_transpose, layer.initial_hidden_state.parameters, layer.weights_hidden.gradient);
                matrix_multiply_accumulate_reduce(device, buffers.buffer, layer.weights_hidden.parameters, layer.initial_hidden_state.gradient);
            }
        }
        else{
            if constexpr(CAN_RESET_SAMPLE){
                if constexpr(CALCULATE_D_PARAMETERS){
#ifdef RL_TOOLS_ENABLE_TRACY
                    ZoneScopedN("gru::CAN_RESET_SAMPLE::d_weights_hidden");
#endif
                    auto output_previous_step = buffers.previous_output_scratch;
                    matrix_multiply_accumulate(device, buffer_transpose, output_previous_step, layer.weights_hidden.gradient);
                }
                auto d_output_previous_step = view(device, d_output, step_i-1);
                if constexpr(CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
                    copy(device, device, buffers.buffer, buffers.buffer2);
                }
                for(TI sample_i = 0; sample_i < BATCH_SIZE; sample_i++){
                    bool reset = nn::layers::gru::mode::reset_sample<LAYER_SPEC>(device, mode, step_i, sample_i);
                    if(reset){
                        auto buffer_sample = view(device, buffers.buffer, sample_i);
                        set_all(device, buffer_sample, 0);
                    }
                    else{
                        if constexpr(CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
                            auto buffer2_sample = view(device, buffers.buffer2, sample_i);
                            set_all(device, buffer2_sample, 0);
                        }
                    }
                }
                matrix_multiply_accumulate(device, buffers.buffer, layer.weights_hidden.parameters, d_output_previous_step);
                if constexpr(CALCULATE_D_PARAMETERS && LAYER_SPEC::LEARN_INITIAL_HIDDEN_STATE){
#ifdef RL_TOOLS_ENABLE_TRACY
                    ZoneScopedN("gru::CAN_RESET_SAMPLE::matrix_multiply_accumulate_reduce");
#endif
                    matrix_multiply_accumulate_reduce(device, buffers.buffer2, layer.weights_hidden.parameters, layer.initial_hidden_state.gradient);
                }
            }
            else{
                if constexpr(CALCULATE_D_PARAMETERS){
                    auto output_previous_step = view(device, layer.output, step_i-1);
                    matrix_multiply_accumulate(device, buffer_transpose, output_previous_step, layer.weights_hidden.gradient);
                }
                auto d_output_previous_step = view(device, d_output, step_i-1);
                matrix_multiply_accumulate(device, buffers.buffer, layer.weights_hidden.parameters, d_output_previous_step);
            }
        }

        if constexpr(CALCULATE_D_PARAMETERS){
#ifdef RL_TOOLS_ENABLE_TRACY
            ZoneScopedN("gru::CAN_RESET_SAMPLE::d_hn_grad");
#endif
            auto b_hn_grad = view_range(device, layer.biases_hidden.gradient, 2*LAYER_SPEC::HIDDEN_DIM, tensor::ViewSpec<0, LAYER_SPEC::HIDDEN_DIM>{});
            auto buffer_n_transpose = permute(device, buffers.buffer_n, tensor::PermutationSpec<1, 0>{});
            reduce_sum<true>(device, buffer_n_transpose, b_hn_grad);
        }
    }
    template<bool CALCULATE_D_INPUT, bool CALCULATE_D_PARAMETERS, typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _backward(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, const Mode<MODE>& mode){
        using TI = typename DEVICE::index_t;
        for(TI step_i=LAYER_SPEC::SEQUENCE_LENGTH-1; true; step_i--){
            _backward<CALCULATE_D_INPUT, CALCULATE_D_PARAMETERS>(device, layer, input, d_output, d_input, buffers, step_i, mode);
            if(step_i == 0){
                break;
            }
        }
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, typename DEVICE::index_t step_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        _backward<true, true>(device, layer, input, d_output, d_input, buffers, step_i, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, typename DEVICE::index_t step_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using T = typename LAYER_SPEC::T;
        using TI = typename DEVICE::index_t;
        Tensor<tensor::Specification<T, TI, typename INPUT_SPEC::SHAPE, true>> d_input_dummy; // not allocated, pointer should be optimized away because it is not used
        _backward<false, true>(device, layer, input, d_output, d_input_dummy, buffers, step_i, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::backward_full");
#endif
        _backward<true, true>(device, layer, input, d_output, d_input, buffers, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
#ifdef RL_TOOLS_ENABLE_TRACY
        ZoneScopedN("gru::backward_full");
#endif
        using T = typename LAYER_SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        Tensor<tensor::Specification<T, TI, typename D_INPUT_SPEC::SHAPE, true>> input_dummy; // not allocated, pointer should be optimized away because it is not used
        _backward<true, false>(device, layer, input_dummy, d_output, d_input, buffers, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::gru::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::gru::buffers::Backward<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using T = typename LAYER_SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename DEVICE::index_t;
        Tensor<tensor::Specification<T, TI, typename INPUT_SPEC::SHAPE, true>> d_input_dummy; // not allocated, pointer should be optimized away because it is not used
        _backward<false, true>(device, layer, input, d_output, d_input_dummy, buffers, mode);
    }

    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::gru::LayerForward<SOURCE_SPEC>& source, nn::layers::gru::LayerForward<TARGET_SPEC>& target){
        copy(source_device, target_device, source.weights_input, target.weights_input);
        copy(source_device, target_device, source.biases_input, target.biases_input);
        copy(source_device, target_device, source.weights_hidden, target.weights_hidden);
        copy(source_device, target_device, source.biases_hidden, target.biases_hidden);
        copy(source_device, target_device, source.initial_hidden_state, target.initial_hidden_state);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::gru::LayerBackward<SOURCE_SPEC>& source, nn::layers::gru::LayerBackward<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::gru::LayerForward<SOURCE_SPEC>&>(source), static_cast<nn::layers::gru::LayerForward<TARGET_SPEC>&>(target));
        if constexpr(tensor::same_dimensions_shape<typename SOURCE_SPEC::INPUT_SHAPE, typename TARGET_SPEC::INPUT_SHAPE>()){
            copy(source_device, target_device, source.post_activation, target.post_activation);
            copy(source_device, target_device, source.n_pre_pre_activation, target.n_pre_pre_activation);
            copy(source_device, target_device, source.output, target.output);
        }
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::gru::LayerGradient<SOURCE_SPEC>& source, nn::layers::gru::LayerGradient<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::gru::LayerBackward<SOURCE_SPEC>&>(source), static_cast<nn::layers::gru::LayerBackward<TARGET_SPEC>&>(target));
    }

    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::gru::LayerForward<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, source.weights_input, target.weights_input);
        copy_from_generic(source_device, target_device, source.biases_input, target.biases_input);
        copy_from_generic(source_device, target_device, source.weights_hidden, target.weights_hidden);
        copy_from_generic(source_device, target_device, source.biases_hidden, target.biases_hidden);
        copy_from_generic(source_device, target_device, source.initial_hidden_state, target.initial_hidden_state);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::gru::LayerBackward<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, static_cast<const typename SOURCE::PARENT&>(source), static_cast<nn::layers::gru::LayerForward<TARGET_SPEC>&>(target));
        if constexpr(tensor::same_dimensions_shape<typename SOURCE::SPEC::INPUT_SHAPE, typename TARGET_SPEC::INPUT_SHAPE>()){
            copy_from_generic(source_device, target_device, source.post_activation, target.post_activation);
            copy_from_generic(source_device, target_device, source.n_pre_pre_activation, target.n_pre_pre_activation);
            copy_from_generic(source_device, target_device, source.output, target.output);
        }
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn::layers::gru::LayerGradient<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, static_cast<const typename SOURCE::SPEC&>(source), static_cast<nn::layers::gru::LayerBackward<TARGET_SPEC>&>(target));
    }

    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::gru::buffers::Evaluation<SOURCE_SPEC>& source, nn::layers::gru::buffers::Evaluation<TARGET_SPEC>& target){
        copy(source_device, target_device, source.post_activation, target.post_activation);
        copy(source_device, target_device, source.n_pre_pre_activation, target.n_pre_pre_activation);
        copy(source_device, target_device, source.step_by_step_output, target.step_by_step_output);
        copy(source_device, target_device, source.previous_output_scratch, target.previous_output_scratch);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const nn::layers::gru::buffers::Backward<SOURCE_SPEC>& source, nn::layers::gru::buffers::Backward<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::gru::buffers::Evaluation<SOURCE_SPEC>&>(source), static_cast<nn::layers::gru::buffers::Evaluation<TARGET_SPEC>&>(target));
        copy(source_device, target_device, source.buffer, target.buffer);
        copy(source_device, target_device, source.buffer2, target.buffer2);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::gru::LayerForward<SPEC>& l) { }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::gru::LayerBackward<SPEC>& l) {
        set_all(device, l.post_activation, 0);
        set_all(device, l.n_pre_pre_activation, 0);
        set_all(device, l.output, 0);
        reset_forward_state(device, static_cast<rl_tools::nn::layers::gru::LayerForward<SPEC>&>(l));
    }

    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::gru::LayerForward<SPEC_1>& l1, const rl_tools::nn::layers::gru::LayerForward<SPEC_2>& l2) {
        typename SPEC_1::TYPE_POLICY::DEFAULT diff = 0;
        diff += abs_diff(device, l1.weights_input, l2.weights_input);
        diff += abs_diff(device, l1.biases_input, l2.biases_input);
        diff += abs_diff(device, l1.weights_hidden, l2.weights_hidden);
        diff += abs_diff(device, l1.biases_hidden, l2.biases_hidden);
        diff += abs_diff(device, l1.initial_hidden_state, l2.initial_hidden_state);
        return diff;
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::gru::LayerBackward<SPEC_1>& l1, const rl_tools::nn::layers::gru::LayerBackward<SPEC_2>& l2) {
        using T = typename SPEC_1::TYPE_POLICY::DEFAULT;
        T diff = abs_diff(device, static_cast<const rl_tools::nn::layers::gru::LayerForward<SPEC_1>&>(l1), static_cast<const rl_tools::nn::layers::gru::LayerForward<SPEC_2>&>(l2));
        diff += abs_diff(device, l1.post_activation, l2.post_activation);
        diff += abs_diff(device, l1.n_pre_pre_activation, l2.n_pre_pre_activation);
        diff += abs_diff(device, l1.output, l2.output);
        return diff;
    }
    template <typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const rl_tools::nn::layers::gru::LayerGradient<SPEC_1>& l1, const rl_tools::nn::layers::gru::LayerGradient<SPEC_2>& l2) {
        return abs_diff(device, static_cast<const rl_tools::nn::layers::gru::LayerBackward<SPEC_1>&>(l1), static_cast<const rl_tools::nn::layers::gru::LayerBackward<SPEC_2>&>(l2));
    }

    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::gru::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool weights_nan = is_nan(device, l.weights_input, mode) || is_nan(device, l.weights_hidden, mode);
        bool biases_nan = is_nan(device, l.biases_input, mode) || is_nan(device, l.biases_hidden, mode);
        bool initial_hidden_state_nan = is_nan(device, l.initial_hidden_state, mode);
        return weights_nan || biases_nan || initial_hidden_state_nan;
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::gru::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::gru::LayerForward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.post_activation, mode) || is_nan(device, l.n_pre_pre_activation, mode) || is_nan(device, l.output, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::gru::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        return is_nan(device, static_cast<const rl_tools::nn::layers::gru::LayerBackward<SPEC>&>(l), mode);
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::gru::LayerForward<SPEC>& layer){
        free(device, layer.weights_input);
        free(device, layer.biases_input);
        free(device, layer.weights_hidden);
        free(device, layer.biases_hidden);
        free(device, layer.initial_hidden_state);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::gru::LayerBackward<SPEC>& layer){
        free(device, static_cast<nn::layers::gru::LayerForward<SPEC>&>(layer));
        free(device, layer.n_pre_pre_activation);
        free(device, layer.post_activation);
        free(device, layer.output);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn::layers::gru::LayerBackward<SPEC>& layer){
        auto tensor_flat = to_tensor(device, layer.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::gru::LayerGradient<SPEC>& layer) {
        return gradient_norm(device, layer.weights_input) + gradient_norm(device, layer.weights_hidden) + gradient_norm(device, layer.biases_input) + gradient_norm(device, layer.biases_hidden) + gradient_norm(device, layer.initial_hidden_state);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif