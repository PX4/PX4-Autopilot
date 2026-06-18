#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_GRU_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_GRU_PERSIST_CODE_H
#include "layer.h"
#include "../../../containers/matrix/persist_code.h"
#include <sstream>
#include "../../../persist/code.h"
#include "../../../containers/matrix/persist_code.h"
#include "../../../nn/capability/persist_code.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    namespace nn::layers::gru::persist_code{
        template<typename DEVICE, typename SPEC>
        rl_tools::persist::Code finish(DEVICE& device, nn::layers::gru::LayerForward<SPEC> &layer, std::string name, rl_tools::persist::Code input, bool const_declaration=true, typename DEVICE::index_t indent=0){
            using TI = typename DEVICE::index_t;
            std::stringstream indent_ss;
            for(TI i=0; i < indent; i++){
                indent_ss << "    ";
            }
            std::string ind = indent_ss.str();
            using TI = typename DEVICE::index_t;
            std::stringstream ss, ss_header;
            ss_header << input.header;
            ss_header << "#include <rl_tools/nn/layers/gru/layer.h>\n";
            ss << input.body;
            std::string TI_string = containers::persist::get_type_string<typename SPEC::TI>();
            ss << ind << "namespace " << name << " {\n";
            ss << ind << "    using TYPE_POLICY = " << to_string(typename SPEC::TYPE_POLICY{}) << ";\n";
            ss << ind << "    using CONFIG = " << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::layers::gru::Configuration<"
               << "TYPE_POLICY, "
               << TI_string << ", "
               << SPEC::HIDDEN_DIM << ", "
               << get_type_string_tag(device, typename SPEC::PARAMETER_GROUP{}) << ", "
               << (SPEC::FAST_TANH ? "true" : "false")
               << ">; \n";
            ss << ind << "    " << "using TEMPLATE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::layers::gru::BindConfiguration<CONFIG>;" << "\n";
            ss << ind << "    " << "using INPUT_SHAPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::Shape<" << TI_string << ", " << get<0>(typename SPEC::INPUT_SHAPE{}) << ", " << get<1>(typename SPEC::INPUT_SHAPE{}) << ", " << get<2>(typename SPEC::INPUT_SHAPE{}) << ">;\n";
            ss << ind << "    " << "using CAPABILITY = " << to_string(typename SPEC::CAPABILITY::template CHANGE_PARAMETERS<true, true>{}) << ";" << "\n";
            ss << ind << "    " << "using TYPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::layers::gru::Layer<CONFIG, CAPABILITY, INPUT_SHAPE>;" << "\n";
            std::string initializer_list;
            std::string forward_members = "{weights_input::parameters, biases_input::parameters, weights_hidden::parameters, biases_hidden::parameters, initial_hidden_state::parameters}";
            std::string backward_members = "{" + forward_members + ", post_activation::container, n_pre_pre_activation::container, output::container}";
            if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Forward){
                initializer_list = forward_members;
            }
            else{
                if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Backward){
                    initializer_list = backward_members;
                }
                else{
                    if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Gradient){
                        initializer_list = "{" + backward_members + "}";
                    }
                    else{
                        utils::assert_exit(device, false, "Unknown capability");
                    }
                }
            }
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "TYPE module = " << initializer_list << ";\n";
            ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory = " << initializer_list << ";" << "\n";
            ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory_function(){return " << initializer_list << ";" << "}\n";
            ss << ind << "}\n";


            return {ss_header.str(), ss.str()};
        }
    }
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::layers::gru::LayerForward<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0, bool finish=true){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        using TI = typename DEVICE::index_t;
        std::stringstream ss, ss_header;
        ss << ind << "namespace " << name << " {\n";
        {
            auto weights_input = save_code_split(device, layer.weights_input, "weights_input", const_declaration, indent+1);
            ss_header << weights_input.header;
            ss << weights_input.body;
        }
        {
            auto weights_hidden = save_code_split(device, layer.weights_hidden, "weights_hidden", const_declaration, indent+1);
            ss_header << weights_hidden.header;
            ss << weights_hidden.body;
        }
        {
            auto biases_input = save_code_split(device, layer.biases_input, "biases_input", const_declaration, indent+1);
            ss_header << biases_input.header;
            ss << biases_input.body;
        }
        {
            auto biases_hidden = save_code_split(device, layer.biases_hidden, "biases_hidden", const_declaration, indent+1);
            ss_header << biases_hidden.header;
            ss << biases_hidden.body;
        }
        {
            auto initial_hidden_state = save_code_split(device, layer.initial_hidden_state, "initial_hidden_state", const_declaration, indent+1);
            ss_header << initial_hidden_state.header;
            ss << initial_hidden_state.body;
        }
        ss << ind << "}\n";
        if(finish){
            return nn::layers::gru::persist_code::finish(device, layer, name, {ss_header.str(), ss.str()}, const_declaration, indent);
        }
        else{
            return {ss_header.str(), ss.str()};
        }
    }
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::layers::gru::LayerBackward<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0, bool finish=true){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        using TI = typename DEVICE::index_t;
        std::stringstream ss, ss_header;
        auto previous = save_code_split(device, static_cast<nn::layers::gru::LayerForward<SPEC>&>(layer), name, const_declaration, indent, false);
        ss_header << previous.header;
        ss << previous.body;
        ss << ind << "namespace " << name << " {\n";
        auto post_activation = save_code_split(device, layer.post_activation, "post_activation", const_declaration, indent+1);
        auto n_pre_pre_activation = save_code_split(device, layer.n_pre_pre_activation, "n_pre_pre_activation", const_declaration, indent+1);
        auto output = save_code_split(device, layer.output, "output", const_declaration, indent+1);
        ss_header << n_pre_pre_activation.header;
        ss_header << post_activation.header;
        ss_header << output.header;
        ss << n_pre_pre_activation.body;
        ss << post_activation.body;
        ss << output.body;
        ss << ind << "}\n";
        if(finish){
            return nn::layers::gru::persist_code::finish(device, layer, name, {ss_header.str(), ss.str()}, const_declaration, indent);
        }
        else{
            return {ss_header.str(), ss.str()};
        }
    }

    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::layers::gru::LayerGradient<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        using TI = typename DEVICE::index_t;
        std::stringstream ss, ss_header;
        auto previous = save_code_split(device, static_cast<nn::layers::gru::LayerBackward<SPEC>&>(layer), name, const_declaration, indent, false);
        ss_header << previous.header;
        ss << previous.body;
        ss << ind << "namespace " << name << " {\n";
        ss << ind << "}\n";
        return nn::layers::gru::persist_code::finish(device, layer, name, {ss_header.str(), ss.str()}, const_declaration, indent);
    }

    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn::layers::gru::LayerForward<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        auto code = save_code_split(device, layer, name, const_declaration, indent);
        return code.header + code.body;
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn::layers::gru::LayerBackward<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        auto code = save_code_split(device, layer, name, const_declaration, indent);
        return code.header + code.body;
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn::layers::gru::LayerGradient<SPEC> &layer, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        auto code = save_code_split(device, layer, name, const_declaration, indent);
        return code.header + code.body;
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn::layers::gru::LayerGradient<SPEC>& layer) {
        std::string data;
        data += "{";
        data += "\"weights_input\": " + nn_analytics(device, layer.weights_input) + ", ";
        data += "\"biases_input\": " + nn_analytics(device, layer.biases_input) + ", ";
        data += "\"weights_hidden\": " + nn_analytics(device, layer.weights_hidden) + ", ";
        data += "\"biases_hidden\": " + nn_analytics(device, layer.biases_hidden) + ", ";
        data += "\"initial_hidden_state\": " + nn_analytics(device, layer.initial_hidden_state) + ", ";
        data += "\"n_pre_pre_activation\": " + json(device, layer.n_pre_pre_activation) + ", ";
        data += "\"post_activation\": " + json(device, layer.post_activation) + ", ";
        data += "\"output\": " + json(device, layer.output);
        data += "}";
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
