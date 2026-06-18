#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_OPERATIONS_GENERIC_H

#include "model.h"
#include "../../utils/generic/typing.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& module){
        malloc(device, module.content);
    }
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& module){
        free(device, module.content);
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::multi_agent_wrapper::ModuleState<STATE_SPEC>& state){
        malloc(device, state.inner_state);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::multi_agent_wrapper::ModuleState<SOURCE_SPEC>& source, nn_models::multi_agent_wrapper::ModuleState<TARGET_SPEC>& target){
        copy(source_device, target_device, source.inner_state, target.inner_state);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename STATE_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model, nn_models::multi_agent_wrapper::ModuleState<STATE_SPEC>& state, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        reset(device, model.content, state.inner_state, rng, mode);
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::multi_agent_wrapper::ModuleState<STATE_SPEC>& state){
        free(device, state.inner_state);
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffer){
        malloc(device, buffer.input);
        malloc(device, buffer.d_input);
        malloc(device, buffer.output);
        malloc(device, buffer.buffer);
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffer){
        free(device, buffer.input);
        free(device, buffer.d_input);
        free(device, buffer.output);
        free(device, buffer.buffer);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_BUFFER_SPEC, typename TARGET_BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::multi_agent_wrapper::ModuleBuffer<SOURCE_BUFFER_SPEC>& source, nn_models::multi_agent_wrapper::ModuleBuffer<TARGET_BUFFER_SPEC>& target){
        copy(source_device, target_device, source.buffer, target.buffer);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& module, RNG& rng){
        init_weights(device, module.content, rng);
    }

    template <typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_first_layer(nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model){
        return get_first_layer(model.content);
    }
    template <typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_first_layer(const nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model){
        return get_first_layer(model.content);
    }

    template <typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_layer(nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model){
        return get_last_layer(model.content);
    }
    template <typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_layer(const nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model){
        return get_last_layer(model.content);
    }

    template <typename BUFFER_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_buffer(nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffer){
        return get_last_buffer(buffer.buffer);
    }
    template <typename BUFFER_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_buffer(const nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffer){
        return get_last_buffer(buffer.buffer);
    }

    template <typename DEVICE, typename SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto output(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& m){
        auto output_reshaped = reshape_row_major(device, output(device, m.content), typename SPEC::OUTPUT_SHAPE{});
        return output_reshaped;
    }
    template <typename DEVICE, typename SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto output(DEVICE& device, const nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& m){
        auto output_reshaped = reshape_row_major(device, output(device, m.content), typename SPEC::OUTPUT_SHAPE{});
        return output_reshaped;
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::SHAPE::LENGTH == 3);
        static_assert(OUTPUT_SPEC::SHAPE::LENGTH == 3);
        constexpr TI INPUT_DIM = INPUT_SPEC::SHAPE::template GET<INPUT_SPEC::SHAPE::LENGTH-1>;
        constexpr TI OUTPUT_DIM = OUTPUT_SPEC::SHAPE::template GET<OUTPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        static_assert(OUTPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 1;
        constexpr TI BATCH_SIZE = INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;

        auto input_buffer_view = view_range(device, buffers.input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto output_buffer_view = view_range(device, buffers.output, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, input, input_buffer_view);// forgot why this? to make it dense for the reshape?
        auto input_inner = reshape_row_major(device, input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto output_inner = reshape_row_major(device, output_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, OUTPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        evaluate(device, model.content, input_inner, output_inner, buffers.buffer, rng, mode);
        copy(device, device, output_buffer_view, output);
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename STATE_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model, const Tensor<INPUT_SPEC>& input, nn_models::multi_agent_wrapper::ModuleState<STATE_SPEC>& state, Tensor<OUTPUT_SPEC>& output, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::SHAPE::LENGTH == 2);
        static_assert(OUTPUT_SPEC::SHAPE::LENGTH == 2);
        constexpr TI INPUT_DIM = INPUT_SPEC::SHAPE::template GET<INPUT_SPEC::SHAPE::LENGTH-1>;
        constexpr TI OUTPUT_DIM = OUTPUT_SPEC::SHAPE::template GET<OUTPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        static_assert(OUTPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 0;
        constexpr TI BATCH_SIZE = INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;
        using TI = typename DEVICE::index_t;

        auto input_buffer_step = view(device, buffers.input, 0);
        auto output_buffer_step = view(device, buffers.output, 0);
        auto input_buffer_view = view_range(device, input_buffer_step, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto output_buffer_view = view_range(device, output_buffer_step, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, input, input_buffer_view);
        auto input_inner = reshape_row_major(device, input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto output_inner = reshape_row_major(device, output_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, OUTPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        evaluate_step(device, model.content, input_inner, state.inner_state, output_inner, buffers.buffer, rng, mode);
        copy(device, device, output_buffer_view, output);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<MODULE_SPEC>& module, const Tensor<INPUT_SPEC>& input, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::SHAPE::LENGTH == 3);
        constexpr TI INPUT_DIM = INPUT_SPEC::SHAPE::template GET<INPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 1;
        constexpr TI BATCH_SIZE = INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;
        using TI = typename DEVICE::index_t;

        auto input_buffer_view = view_range(device, buffers.input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto output_buffer_view = view_range(device, buffers.output, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, input, input_buffer_view);
        auto input_inner = reshape_row_major(device, input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        forward(device, module.content, input_inner, buffers.buffer, rng, mode);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<MODULE_SPEC>& module, INPUT& input, OUTPUT& output, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward(device, module, input, buffers, rng, mode);
        auto module_output = rl_tools::output(device, module);
        copy(device, device, module_output, output);
    }
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<MODULE_SPEC>& module){
        zero_gradient(device, module.content);
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& module, OPTIMIZER& optimizer) {
        _reset_optimizer_state(device, module.content, optimizer);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& module) {
        reset_forward_state(device, module.content);
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<MODULE_SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC> buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::SHAPE::LENGTH == 3);
        static_assert(D_INPUT_SPEC::SHAPE::LENGTH == 3);
        static_assert(D_OUTPUT_SPEC::SHAPE::LENGTH == 3);
        constexpr TI INPUT_DIM = INPUT_SPEC::SHAPE::template GET<INPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM == D_INPUT_SPEC::SHAPE::template GET<D_INPUT_SPEC::SHAPE::LENGTH-1>);
        constexpr TI OUTPUT_DIM = D_OUTPUT_SPEC::SHAPE::template GET<D_OUTPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        static_assert(OUTPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 1;
        constexpr TI BATCH_SIZE = INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;
        static_assert(BATCH_SIZE == D_INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>);
        static_assert(BATCH_SIZE == D_OUTPUT_SPEC::SHAPE::template GET<BATCH_AXIS>);
        using TI = typename DEVICE::index_t;

        auto input_buffer_view = view_range(device, buffers.input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto d_input_buffer_view = view_range(device, buffers.d_input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto d_output_buffer_view = view_range(device, buffers.output, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, input, input_buffer_view);
        copy(device, device, d_output, d_output_buffer_view);
        auto input_inner = reshape_row_major(device, input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto d_input_inner = reshape_row_major(device, d_input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto d_output_inner = reshape_row_major(device, d_output_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, OUTPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        backward_full(device, model.content, input_inner, d_output_inner, d_input_inner, buffers.buffer, mode);
        copy(device, device, d_input_buffer_view, d_input);
    }
    template<typename DEVICE, typename MODULE_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn_models::multi_agent_wrapper::ModuleBackward<MODULE_SPEC>& model, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC> buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(D_INPUT_SPEC::SHAPE::LENGTH == 3);
        static_assert(D_OUTPUT_SPEC::SHAPE::LENGTH == 3);
        constexpr TI INPUT_DIM = D_INPUT_SPEC::SHAPE::template GET<D_INPUT_SPEC::SHAPE::LENGTH-1>;
        constexpr TI OUTPUT_DIM = D_OUTPUT_SPEC::SHAPE::template GET<D_OUTPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        static_assert(OUTPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 1;
        constexpr TI BATCH_SIZE = D_INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;
        static_assert(BATCH_SIZE == D_OUTPUT_SPEC::SHAPE::template GET<BATCH_AXIS>);
        using TI = typename DEVICE::index_t;

        auto d_input_buffer_view = view_range(device, buffers.d_input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto d_output_buffer_view = view_range(device, buffers.output, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, d_output, d_output_buffer_view);
        auto d_input_inner = reshape_row_major(device, d_input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto d_output_inner = reshape_row_major(device, d_output_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, OUTPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        backward_input(device, model.content, d_output_inner, d_input_inner, buffers.buffer, mode);
        copy(device, device, d_input_buffer_view, d_input);
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<MODULE_SPEC>& model, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC> buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using TI = typename DEVICE::index_t;
        static_assert(INPUT_SPEC::SHAPE::LENGTH == 3);
        static_assert(D_OUTPUT_SPEC::SHAPE::LENGTH == 3);
        constexpr TI INPUT_DIM = INPUT_SPEC::SHAPE::template GET<INPUT_SPEC::SHAPE::LENGTH-1>;
        constexpr TI OUTPUT_DIM = D_OUTPUT_SPEC::SHAPE::template GET<D_OUTPUT_SPEC::SHAPE::LENGTH-1>;
        static_assert(INPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        static_assert(OUTPUT_DIM % MODULE_SPEC::N_AGENTS == 0);
        constexpr TI BATCH_AXIS = 1;
        constexpr TI BATCH_SIZE = INPUT_SPEC::SHAPE::template GET<BATCH_AXIS>;
        static_assert(BATCH_SIZE == D_OUTPUT_SPEC::SHAPE::template GET<BATCH_AXIS>);
        using TI = typename DEVICE::index_t;

        auto input_buffer_view = view_range(device, buffers.input, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        auto d_output_buffer_view = view_range(device, buffers.output, tensor::ViewSpec<BATCH_AXIS, BATCH_SIZE>{});
        copy(device, device, input, input_buffer_view);
        copy(device, device, d_output, d_output_buffer_view);
        auto input_inner = reshape_row_major(device, input_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, INPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        auto d_output_inner = reshape_row_major(device, d_output_buffer_view, tensor::Shape<TI, BATCH_SIZE*MODULE_SPEC::N_AGENTS, OUTPUT_DIM/MODULE_SPEC::N_AGENTS>{});
        backward(device, model.content, input_inner, d_output_inner, buffers.buffer, mode);
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& model, OPTIMIZER& optimizer) {
        update(device, model.content, optimizer);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn_models::multi_agent_wrapper::ModuleForward<SOURCE_SPEC>& source, nn_models::multi_agent_wrapper::ModuleForward<TARGET_SPEC>& target){
        copy(source_device, target_device, source.content, target.content);
    }

    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::T abs_diff(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<SPEC_A>& a, const nn_models::multi_agent_wrapper::ModuleForward<SPEC_B>& b){
        auto diff = abs_diff(device, a.content, b.content);
        return diff;
    }


    template<typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<MODULE_SPEC>& model){
        bool current_module_nan = is_nan(device, model.content);
        return current_module_nan;
    }

    template <typename DEVICE, typename BUFFER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn_models::multi_agent_wrapper::ModuleBuffer<BUFFER_SPEC>& buffer, RNG& rng){
        sample(device, buffer.buffer, rng);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void print(DEVICE& device, const nn_models::multi_agent_wrapper::ModuleForward<SPEC>& model, typename DEVICE::index_t layer_i = 0){
        print(device, model.content, layer_i);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
