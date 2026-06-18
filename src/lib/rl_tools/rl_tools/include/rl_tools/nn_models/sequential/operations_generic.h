#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_SEQUENTIAL_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_SEQUENTIAL_OPERATIONS_GENERIC_H

#include "model.h"
#include "../../utils/generic/typing.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::sequential::ModuleForward<MODULE_SPEC>& module){
        using namespace nn_models::sequential;
        malloc(device, module.content);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, OutputModule>){
            malloc(device, module.next_module);
        }
    }
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::sequential::ModuleForward<MODULE_SPEC>& module){
        using namespace nn_models::sequential;
        free(device, module.content);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, OutputModule>){
            free(device, module.next_module);
        }
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::sequential::ContentState<STATE_SPEC>& content_state){
        using namespace nn_models::sequential;
        malloc(device, content_state.state);
        if constexpr(!utils::typing::is_same_v<typename STATE_SPEC::NEXT_SPEC, OutputModule>){
            malloc(device, content_state.next_content_state);
        }
    }
    template <typename DEVICE, typename MODULE_SPEC, typename STATE_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, nn_models::sequential::ContentState<STATE_SPEC>& content_state, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using namespace nn_models::sequential;
        reset(device, model.content, content_state.state, rng, mode);
        if constexpr(!utils::typing::is_same_v<typename STATE_SPEC::NEXT_SPEC, OutputModule>){
            reset(device, model.next_module, content_state.next_content_state, rng, mode);
        }
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::sequential::ContentState<STATE_SPEC>& content_state){
        using namespace nn_models::sequential;
        free(device, content_state.state);
        if constexpr(!utils::typing::is_same_v<typename STATE_SPEC::NEXT_SPEC, OutputModule>){
            free(device, content_state.next_content_state);
        }
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_STATE_SPEC, typename TARGET_STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::sequential::ContentState<SOURCE_STATE_SPEC>& source, nn_models::sequential::ContentState<TARGET_STATE_SPEC>& target){
        using namespace nn_models::sequential;
        copy(source_device, target_device, source.state, target.state);
        if constexpr(!utils::typing::is_same_v<typename TARGET_STATE_SPEC::NEXT_SPEC, OutputModule>){
            copy(source_device, target_device, source.next_content_state, target.next_content_state);
        }
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::sequential::ModuleState<STATE_SPEC>& state){
        malloc(device, state.content_state);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename STATE_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, nn_models::sequential::ModuleState<STATE_SPEC>& state, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        reset(device, model, state.content_state, rng, mode);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::sequential::ModuleState<SOURCE_SPEC>& source, nn_models::sequential::ModuleState<TARGET_SPEC>& target){
        copy(source_device, target_device, source.content_state, target.content_state);
    }
    template <typename DEVICE, typename STATE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::sequential::ModuleState<STATE_SPEC>& state){
        free(device, state.content_state);
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::sequential::ContentBuffer<BUFFER_SPEC>& content_buffer){
        using namespace nn_models::sequential;
        malloc(device, content_buffer.buffer);
        if constexpr(!utils::typing::is_same_v<typename BUFFER_SPEC::NEXT_SPEC, OutputModule>){
            malloc(device, content_buffer.next_content_buffer);
        }
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::sequential::ContentBuffer<BUFFER_SPEC>& content_buffer){
        using namespace nn_models::sequential;
        free(device, content_buffer.buffer);
        if constexpr(!utils::typing::is_same_v<typename BUFFER_SPEC::NEXT_SPEC, OutputModule>){
            free(device, content_buffer.next_content_buffer);
        }
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_BUFFER_SPEC, typename TARGET_BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::sequential::ContentBuffer<SOURCE_BUFFER_SPEC>& source, nn_models::sequential::ContentBuffer<TARGET_BUFFER_SPEC>& target){
        using namespace nn_models::sequential;
        copy(source_device, target_device, source.buffer, target.buffer);
        if constexpr(!utils::typing::is_same_v<typename TARGET_BUFFER_SPEC::NEXT_SPEC, OutputModule>){
            copy(source_device, target_device, source.next_content_buffer, target.next_content_buffer);
        }
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers){
        malloc(device, buffers.tick);
        malloc(device, buffers.tock);
        malloc(device, buffers.content_buffer);
    }
    template <typename DEVICE, typename BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers){
        free(device, buffers.tick);
        free(device, buffers.tock);
        free(device, buffers.content_buffer);
    }
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_BUFFER_SPEC, typename TARGET_BUFFER_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn_models::sequential::ModuleBuffer<SOURCE_BUFFER_SPEC>& source, nn_models::sequential::ModuleBuffer<TARGET_BUFFER_SPEC>& target){
        copy(source_device, target_device, source.tick, target.tick);
        copy(source_device, target_device, source.tock, target.tock);
        copy(source_device, target_device, source.content_buffer, target.content_buffer);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn_models::sequential::ModuleForward<MODULE_SPEC>& module, RNG& rng){
        using namespace nn_models::sequential;
        init_weights(device, module.content, rng);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, OutputModule>){
            init_weights(device, module.next_module, rng);
        }
    }
    namespace nn_models::sequential{
        template <typename SPEC>
        RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI num_layers(){
            if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
                return num_layers<typename SPEC::NEXT_MODULE::SPEC>() + 1;
            }
            else{
                return 1;
            }
        }
    }
    template <typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI num_layers(const nn_models::sequential::ModuleForward<SPEC>&){
        return nn_models::sequential::num_layers<SPEC>();
    }
    template <typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI num_layers(const nn_models::sequential::ContentBuffer<SPEC>&){
        return nn_models::sequential::num_layers<SPEC>();
    }
    template <typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI num_layers(const nn_models::sequential::ModuleBuffer<SPEC>& buffer){
        return num_layers(buffer.content_buffer);
    }

    template<auto LAYER_I, typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_layer(nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        static_assert(LAYER_I >= 0);
        static_assert(LAYER_I < nn_models::sequential::num_layers<MODULE_SPEC>());
        if constexpr(LAYER_I == 0){
            return model.content;
        }
        else{
            return get_layer<LAYER_I - 1>(model.next_module);
        }
    }
    template<auto LAYER_I, typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_layer(const nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        static_assert(LAYER_I >= 0);
        static_assert(LAYER_I < nn_models::sequential::num_layers<MODULE_SPEC>());
        if constexpr(LAYER_I == 0){
            return model.content;
        }
        else{
            return get_layer<LAYER_I - 1>(model.next_module);
        }
    }
    template <typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_first_layer(nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        return model.content;
    }
    template <typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_first_layer(const nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        return model.content;
    }
    template <typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_layer(nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        return get_layer<nn_models::sequential::num_layers<MODULE_SPEC>()-1>(model);
    }
    template <typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_layer(const nn_models::sequential::ModuleForward<MODULE_SPEC>& model){
        return get_layer<nn_models::sequential::num_layers<MODULE_SPEC>()-1>(model);
    }

    template<auto LAYER_I, typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_buffer(nn_models::sequential::ContentBuffer<MODULE_SPEC>& buffer){
        static_assert(LAYER_I >= 0);
        static_assert(LAYER_I < nn_models::sequential::num_layers<MODULE_SPEC>());
        if constexpr(LAYER_I == 0){
            return buffer.buffer;
        }
        else{
            return get_buffer<LAYER_I - 1>(buffer.next_content_buffer);
        }
    }
    template<auto LAYER_I, typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_buffer(const nn_models::sequential::ContentBuffer<MODULE_SPEC>& buffer){
        static_assert(LAYER_I >= 0);
        static_assert(LAYER_I < nn_models::sequential::num_layers<MODULE_SPEC>());
        if constexpr(LAYER_I == 0){
            return buffer.buffer;
        }
        else{
            return get_buffer<LAYER_I - 1>(buffer.next_content_buffer);
        }
    }
    template<auto LAYER_I, typename MODULE_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_buffer(nn_models::sequential::ModuleBuffer<MODULE_SPEC>& buffer){
        return get_buffer<LAYER_I>(buffer.content_buffer);
    }
    template<auto LAYER_I, typename MODULE_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_buffer(const nn_models::sequential::ModuleBuffer<MODULE_SPEC>& buffer){
        return get_buffer<LAYER_I>(buffer.content_buffer);
    }
    template <typename BUFFER_SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_buffer(nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffer){
        return get_buffer<nn_models::sequential::num_layers<typename BUFFER_SPEC::SPEC>()-1>(buffer);
    }
    template <typename BUFFER_SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto& get_last_buffer(const nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffer){
        return get_buffer<nn_models::sequential::num_layers<typename BUFFER_SPEC::SPEC>()-1>(buffer);
    }

    template <typename TARGET_SHAPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto _content_output_helper(DEVICE& device, Tensor<SPEC>& tensor){
        return tensor;
    }
    template <typename TARGET_SHAPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto _content_output_helper(DEVICE& device, const Tensor<SPEC>& tensor){
        return tensor;
    }
    template <typename TARGET_SHAPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto _content_output_helper(DEVICE& device, Matrix<SPEC>& matrix){
        auto output_tensor = to_tensor(device, matrix);
        auto output_tensor_reshaped = reshape_row_major(device, output_tensor, TARGET_SHAPE{});
        return output_tensor_reshaped;
    }
    template <typename TARGET_SHAPE, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto _content_output_helper(DEVICE& device, const Matrix<SPEC>& matrix){
        auto output_tensor = to_tensor(device, matrix);
        auto output_tensor_reshaped = reshape_row_major(device, output_tensor, TARGET_SHAPE{});
        return output_tensor_reshaped;
    }

    template <typename DEVICE, typename SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT auto content_output(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& m){
        auto output_matrix = output(device, m.content);
        static_assert(sizeof(output_matrix) <= sizeof(void*)); // we don't want to return static matrices by value here.
        return _content_output_helper<typename SPEC::CONTENT::OUTPUT_SHAPE>(device, output_matrix);
    }

    template <typename DEVICE, typename SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT auto content_output(DEVICE& device, const nn_models::sequential::ModuleGradient<SPEC>& m){
        auto output_matrix = output(device, m.content);
        static_assert(sizeof(output_matrix) <= sizeof(void*)); // we don't want to return static matrices by value here
        return _content_output_helper<typename SPEC::CONTENT::OUTPUT_SHAPE>(device, output_matrix);
    }

    template <typename DEVICE, typename SPEC> // non-const
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& m){
        if constexpr (utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            return content_output(device, m);
        } else {
            return output(device, m.next_module);
        }
    }
    template <typename DEVICE, typename SPEC> // const
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, const nn_models::sequential::ModuleGradient<SPEC>& m){
        if constexpr (utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            return content_output(device, m);
        } else {
            return output(device, m.next_module);
        }
    }
    // Evaluate is like a forward pass but without saving intermediate activations (so a backward pass is not possible). Hence we can reuse the memory of the intermediate outputs and just require a double buffer where each buffer has to be able to contain the maximum hidden dimension of the module
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename BUFFER_SPEC, typename CONTENT_BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _evaluate(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, const INPUT& input, OUTPUT& output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, nn_models::sequential::ContentBuffer<CONTENT_BUFFER_SPEC>& content_buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
//        static_assert(nn_models::sequential::buffer_compatible<BUFFER_SPEC, MODULE_SPEC>);
        using TI = typename DEVICE::index_t;
        using DOUBLE_BUFFER_TYPE = decltype(buffers.tick);

        if constexpr(utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            evaluate(device, model.content, input, output, content_buffer.buffer, rng, mode);
        }
        else{
            DOUBLE_BUFFER_TYPE& output_buffer = TICK ? buffers.tick : buffers.tock;
//            auto output_buffer_view = view(device, output_buffer, matrix::ViewSpec<BATCH_SIZE, MODULE_SPEC::CONTENT::OUTPUT_DIM>{});
            constexpr TI BATCH_SIZE = get<1>(typename INPUT::SPEC::SHAPE{});
//            // todo: this is hard-coded, we need some mechanism to communicate the desired sequence length
            using OUTPUT_SHAPE = typename MODULE_SPEC::CONTENT::template OUTPUT_SHAPE_FACTORY<typename INPUT::SPEC::SHAPE>;
            auto output_buffer_view = view_memory<OUTPUT_SHAPE>(device, output_buffer);
            evaluate(device, model.content, input, output_buffer_view, content_buffer.buffer, rng, mode);
            _evaluate<!TICK>(device, model.next_module, output_buffer_view, output, buffers, content_buffer.next_content_buffer, rng, mode);
        }
    }
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, const INPUT& input, OUTPUT& output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        _evaluate<TICK>(device, model, input, output, buffers, buffers.content_buffer, rng, mode);
    }
    // Evaluate is like a forward pass but without saving intermediate activations (so a backward pass is not possible). Hence we can reuse the memory of the intermediate outputs and just require a double buffer where each buffer has to be able to contain the maximum hidden dimension of the module
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename STATE_SPEC, typename CONTENT_STATE_SPEC, typename BUFFER_SPEC, typename CONTENT_BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _evaluate_step(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, const INPUT& input, nn_models::sequential::ModuleState<STATE_SPEC>& state, nn_models::sequential::ContentState<CONTENT_STATE_SPEC>& content_state, OUTPUT& output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, nn_models::sequential::ContentBuffer<CONTENT_BUFFER_SPEC>& content_buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
//        static_assert(nn_models::sequential::buffer_compatible<BUFFER_SPEC, MODULE_SPEC>);
        using TI = typename DEVICE::index_t;
        using DOUBLE_BUFFER_TYPE = decltype(buffers.tick);

        if constexpr(utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            evaluate_step(device, model.content, input, content_state.state, output, content_buffer.buffer, rng, mode);
        }
        else{
            DOUBLE_BUFFER_TYPE& output_buffer = TICK ? buffers.tick : buffers.tock;
            using OUTPUT_SHAPE = typename MODULE_SPEC::CONTENT::template OUTPUT_SHAPE_FACTORY<typename INPUT::SPEC::SHAPE>;
            static_assert(length(typename INPUT::SPEC::SHAPE{}) == 2);
            static_assert(length(OUTPUT_SHAPE{}) == 2);
            auto output_buffer_view = view_memory<OUTPUT_SHAPE>(device, output_buffer);
            evaluate_step(device, model.content, input, content_state.state, output_buffer_view, content_buffer.buffer, rng, mode);
            _evaluate_step<!TICK>(device, model.next_module, output_buffer_view, state, content_state.next_content_state, output, buffers, content_buffer.next_content_buffer, rng, mode);
        }
    }
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename STATE_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn_models::sequential::ModuleForward<MODULE_SPEC>& model, const INPUT& input, nn_models::sequential::ModuleState<STATE_SPEC>& state, OUTPUT& output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        static_assert(length(typename INPUT::SPEC::SHAPE{}) == 2, "evaluate_step does not have a squence dimension (only batch_size x input_dim)");
        _evaluate_step<TICK>(device, model, input, state, state.content_state, output, buffers, buffers.content_buffer, rng, mode);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename INPUT, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _forward(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& module, INPUT& input, nn_models::sequential::ContentBuffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward(device, module.content, input, buffer.buffer, rng, mode);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            auto output = rl_tools::content_output(device, module);
            _forward(device, module.next_module, output, buffer.next_content_buffer, rng, mode);
        }
    }
    template <typename DEVICE, typename MODULE_SPEC, typename INPUT, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& module, INPUT& input, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        _forward(device, module, input, buffers.content_buffer, rng, mode);
    }
    template <typename DEVICE, typename MODULE_SPEC, typename INPUT, typename OUTPUT, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& module, INPUT& input, OUTPUT& output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        forward(device, module, input, buffers, rng, mode);
        auto output_tensor = rl_tools::output(device, module);
        using MODULE = nn_models::sequential::ModuleGradient<MODULE_SPEC>;
//        auto output_tensor_reshaped = reshape_row_major(device, output_tensor, typename MODULE::OUTPUT_SHAPE{});
        copy(device, device, output_tensor, output);
    }
    template <typename DEVICE, typename MODULE_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& module){
        zero_gradient(device, module.content);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            zero_gradient(device, module.next_module);
        }
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& module, OPTIMIZER& optimizer) {
        _reset_optimizer_state(device, module.content, optimizer);
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            _reset_optimizer_state(device, module.next_module, optimizer);
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& module) {
        reset_forward_state(device, module.content);
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            reset_forward_state(device, module.next_module);
        }
    }
    // the _xxx are unrolling the content_buffers (which should not be exposed to the user)
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename INPUT, typename D_OUTPUT, typename D_INPUT, typename BUFFER_SPEC, typename CONTENT_BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _backward_full(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& model, const INPUT& input, D_OUTPUT& d_output, D_INPUT& d_input, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, nn_models::sequential::ContentBuffer<CONTENT_BUFFER_SPEC>& content_buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        static_assert(nn_models::sequential::buffer_compatible<BUFFER_SPEC, MODULE_SPEC>);
        using TI = typename DEVICE::index_t;
        using DOUBLE_BUFFER_TYPE = decltype(buffers.tick);

        if constexpr(utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            backward_full(device, model.content, input, d_output, d_input, content_buffer.buffer, mode);
        }
        else{
            DOUBLE_BUFFER_TYPE& current_d_output_buffer = TICK ? buffers.tick : buffers.tock;
//            auto current_d_output_buffer_view = view(device, current_d_output_buffer, matrix::ViewSpec<BATCH_SIZE, MODULE_SPEC::CONTENT::OUTPUT_DIM>{});
            auto current_d_output_buffer_view = view_memory<typename MODULE_SPEC::CONTENT::OUTPUT_SHAPE>(device, current_d_output_buffer);
            auto current_output = output(device, model.content);
            auto current_output_tensor = to_tensor(device, current_output);
            _backward_full<!TICK>(device, model.next_module, current_output_tensor, d_output, current_d_output_buffer_view, buffers, content_buffer.next_content_buffer, mode);
            backward_full(device, model.content, input, current_d_output_buffer_view, d_input, content_buffer.buffer, mode);
        }
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT, typename D_OUTPUT, typename D_INPUT, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& model, const INPUT& input, D_OUTPUT& d_output, D_INPUT& d_input, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        _backward_full(device, model, input, d_output, d_input, buffers, buffers.content_buffer, mode);
    }
    template<bool TICK = true, typename DEVICE, typename MODULE_SPEC, typename D_OUTPUT, typename D_INPUT, typename BUFFER_SPEC, typename CONTENT_BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void _backward_input(DEVICE& device, nn_models::sequential::ModuleBackward<MODULE_SPEC>& model, D_OUTPUT& d_output, D_INPUT& d_input, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, nn_models::sequential::ContentBuffer<CONTENT_BUFFER_SPEC>& content_buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(nn_models::sequential::buffer_compatible<BUFFER_SPEC, MODULE_SPEC>);
        using TI = typename DEVICE::index_t;
        using DOUBLE_BUFFER_TYPE = decltype(buffers.tick);

        if constexpr(utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            backward_input(device, model.content, d_output, d_input, content_buffer.buffer, mode);
        }
        else{
            DOUBLE_BUFFER_TYPE& current_d_output_buffer = TICK ? buffers.tick : buffers.tock;
            auto current_d_output_buffer_view = view_memory<typename MODULE_SPEC::CONTENT::OUTPUT_SHAPE>(device, current_d_output_buffer);
//            auto current_d_output_buffer_view = view(device, current_d_output_buffer, matrix::ViewSpec<BATCH_SIZE, MODULE_SPEC::CONTENT::OUTPUT_DIM>{});
            _backward_input<!TICK>(device, model.next_module, d_output, current_d_output_buffer_view, buffers, content_buffer.next_content_buffer);
            backward_input(device, model.content, current_d_output_buffer_view, d_input, content_buffer.buffer);
        }
    }
    template<typename DEVICE, typename MODULE_SPEC, typename D_OUTPUT, typename D_INPUT, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, nn_models::sequential::ModuleBackward<MODULE_SPEC>& model, D_OUTPUT& d_output, D_INPUT& d_input, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        _backward_input(device, model, d_output, d_input, buffers, buffers.content_buffer, mode);
    }
    template<typename DEVICE, typename MODULE_SPEC, typename INPUT, typename D_OUTPUT, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn_models::sequential::ModuleGradient<MODULE_SPEC>& model, const INPUT& input, D_OUTPUT& d_output, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        constexpr bool NEXT_IS_FINAL = utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>;
        using TI = typename DEVICE::index_t;
        // This backward function is called on the final, complete module, the following are called for each submodule, hence the full backward only for the next module (to save the calc for d_input)
        if constexpr(!NEXT_IS_FINAL){
//            auto current_d_input_buffer_view = view(device, buffers.tick, matrix::ViewSpec<BATCH_SIZE, MODULE_SPEC::CONTENT::OUTPUT_DIM>{});
            auto current_d_input_buffer_view = view_memory<typename MODULE_SPEC::CONTENT::OUTPUT_SHAPE>(device, buffers.tick);
            _backward_full<false>(device, model.next_module, content_output(device, model), d_output, current_d_input_buffer_view, buffers, buffers.content_buffer.next_content_buffer, mode);
            backward(device, model.content, input, current_d_input_buffer_view, buffers.content_buffer.buffer, mode);
        }
        else{
            backward(device, model.content, input, d_output, buffers.content_buffer.buffer, mode);
        }
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& model, OPTIMIZER& optimizer) {
        update(device, model.content, optimizer);
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            update(device, model.next_module, optimizer);
        }
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn_models::sequential::ModuleForward<SOURCE_SPEC>& source, nn_models::sequential::ModuleForward<TARGET_SPEC>& target){
        copy(source_device, target_device, source.content, target.content);
        if constexpr(!utils::typing::is_same_v<typename TARGET_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            copy(source_device, target_device, source.next_module, target.next_module);
        }
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE,  typename SOURCE, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy_from_generic(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const SOURCE& source, nn_models::sequential::ModuleForward<TARGET_SPEC>& target){
        copy_from_generic(source_device, target_device, source.content, target.content);
        if constexpr(!utils::typing::is_same_v<typename TARGET_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            copy_from_generic(source_device, target_device, source.next_module, target.next_module);
        }
    }

    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_A::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, nn_models::sequential::ModuleForward<SPEC_A>& a, const nn_models::sequential::ModuleForward<SPEC_B>& b){
        auto diff = abs_diff(device, a.content, b.content);
        if constexpr(!utils::typing::is_same_v<typename SPEC_A::NEXT_MODULE, nn_models::sequential::OutputModule>){
            diff += abs_diff(device, a.next_module, b.next_module);
        }
        return diff;
    }


    template<typename DEVICE, typename MODULE_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, nn_models::sequential::ModuleForward<MODULE_SPEC>& model, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        bool current_module_nan = is_nan(device, model.content, mode);
        if constexpr(!utils::typing::is_same_v<typename MODULE_SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            current_module_nan = current_module_nan || is_nan(device, model.next_module, mode);
        }
        return current_module_nan;
    }

    template <typename DEVICE, typename BUFFER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn_models::sequential::ContentBuffer<BUFFER_SPEC>& buffers, RNG& rng){
        using BUFFER = nn_models::sequential::ContentBuffer<BUFFER_SPEC>;
        sample(device, buffers.buffer, rng);
        if constexpr(!utils::typing::is_same_v<typename BUFFER::NEXT_CONTENT_BUFFER, nn_models::sequential::OutputModule>){
            sample(device, buffers.next_content_buffer, rng);
        }
    }
    template <typename DEVICE, typename BUFFER_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn_models::sequential::ModuleBuffer<BUFFER_SPEC>& buffers, RNG& rng){
        sample(device, buffers.content_buffer, rng);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void print(DEVICE& device, const nn_models::sequential::ModuleForward<SPEC>& model, typename DEVICE::index_t layer_i = 0){
        using TI = typename DEVICE::index_t;
        using LAYER_TYPE = decltype(model.content);
        log(device, device.logger, "Layer ", layer_i, ": ", LAYER_TYPE::INPUT_DIM, " => ", LAYER_TYPE::OUTPUT_DIM);
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            print(device, model.next_module, layer_i + 1);
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn_models::sequential::ModuleForward<SPEC>& model, bool initial = true){
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        T return_value = gradient_norm(device, model.content);
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>) {
            return_value += gradient_norm(device, model.next_module, false);
        }
        if(initial) {
            return_value = math::sqrt(device.math, return_value);
        }
        return return_value;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
