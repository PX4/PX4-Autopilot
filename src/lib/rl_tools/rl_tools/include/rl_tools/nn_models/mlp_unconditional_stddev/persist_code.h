#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MLP_UNCONDITIONAL_STDDEV_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MLP_UNCONDITIONAL_STDDEV_PERSIST_CODE_H



#include <string>
#include <sstream>
#include "../../persist/code.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, template <typename> typename BASE>
    persist::Code save_code_split(DEVICE& device, nn_models::mlp_unconditional_stddev::NeuralNetworkForward<SPEC, BASE>& network, std::string name, bool const_declaration=true, typename DEVICE::index_t indent = 0){
        // using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss_header;
        std::stringstream ss;
        ss << ind << "namespace " << name << " {\n";
        auto input_layer = save_code_split(device, network.input_layer, "input_layer", const_declaration, indent+1);
        ss_header << input_layer.header;
        ss_header << "#include <rl_tools/nn_models/mlp_unconditional_stddev/network.h>\n";
        ss << input_layer.body;
        for(TI hidden_layer_i = 0; hidden_layer_i < SPEC::NUM_HIDDEN_LAYERS; hidden_layer_i++){
            auto hidden_layer = save_code_split(device, network.hidden_layers[hidden_layer_i], "hidden_layer_" + std::to_string(hidden_layer_i), const_declaration, indent+1);
            ss_header << hidden_layer.header;
            ss << hidden_layer.body;
        }
        auto output_layer = save_code_split(device, network.output_layer, "output_layer", const_declaration, indent+1);
        ss_header << output_layer.header;
        ss << output_layer.body;
        auto log_std = save_code_split(device, network.log_std, "log_std", const_declaration, indent+1);
        ss_header << log_std.header;
        ss << log_std.body;
        // std::string T_string = containers::persist::get_type_string<T>();
        std::string TI_string = containers::persist::get_type_string<TI>();
        ss << ind << "    using TYPE_POLICY = " + to_string(typename SPEC::TYPE_POLICY{}) + ";" << "\n";
        ss << ind << "    using CONFIG = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::mlp::Configuration<";
        ss << "TYPE_POLICY, ";
        ss << TI_string << ", ";
        ss << SPEC::OUTPUT_DIM << ", " << SPEC::NUM_LAYERS << ", " << SPEC::HIDDEN_DIM << ", ";
        ss << nn::layers::dense::persist::get_activation_function_string<SPEC::HIDDEN_ACTIVATION_FUNCTION>() << ", ";
        ss << nn::layers::dense::persist::get_activation_function_string<SPEC::OUTPUT_ACTIVATION_FUNCTION>() << ", ";
        ss << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::layers::dense::DefaultInitializer<" << "TYPE_POLICY, " << TI_string << ">";
        ss << ">; \n";
        ss << ind << "    " << "using TEMPLATE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::mlp_unconditional_stddev::BindConfiguration<CONFIG>;" << "\n";
        ss << ind << "    " << "using INPUT_SHAPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::Shape<" << TI_string << ", " << get<0>(typename SPEC::INPUT_SHAPE{}) << ", " << get<1>(typename SPEC::INPUT_SHAPE{}) << ", " << get<2>(typename SPEC::INPUT_SHAPE{}) << ">;\n";
        ss << ind << "    " << "using CAPABILITY = " << to_string(typename SPEC::CAPABILITY::template CHANGE_PARAMETERS<true, true>{}) << ";" << "\n";
        ss << ind << "    " << "using TYPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::mlp_unconditional_stddev::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;" << "\n";
        std::stringstream ss_initializer_list;
        {
            ss_initializer_list << "{{input_layer::factory<TYPE::SPEC::INPUT_LAYER>, ";
            ss_initializer_list << "{";
            for(TI hidden_layer_i = 0; hidden_layer_i < SPEC::NUM_HIDDEN_LAYERS; hidden_layer_i++){
                if(hidden_layer_i > 0){
                    ss_initializer_list << ", ";
                }
                ss_initializer_list << "hidden_layer_" << hidden_layer_i << "::factory<TYPE::SPEC::HIDDEN_LAYER>";
            }
            ss_initializer_list << "}, ";
            ss_initializer_list << "output_layer::factory<TYPE::SPEC::OUTPUT_LAYER>}";
            ss_initializer_list << ", log_std::parameters}";
        }
        std::stringstream ss_initializer_list_create, ss_initializer_list_create_function;
        {
            ss_initializer_list_create << "{{input_layer::factory<typename T_TYPE::SPEC::INPUT_LAYER>, ";
            ss_initializer_list_create_function << "{{input_layer::factory_function<typename T_TYPE::SPEC::INPUT_LAYER>(), ";
            ss_initializer_list_create << "{";
            ss_initializer_list_create_function << "{";
            for(TI hidden_layer_i = 0; hidden_layer_i < SPEC::NUM_HIDDEN_LAYERS; hidden_layer_i++){
                if(hidden_layer_i > 0){
                    ss_initializer_list_create << ", ";
                    ss_initializer_list_create_function << ", ";
                }
                ss_initializer_list_create << "hidden_layer_" << hidden_layer_i << "::factory<typename T_TYPE::SPEC::HIDDEN_LAYER>";
                ss_initializer_list_create_function << "hidden_layer_" << hidden_layer_i << "::factory_function<typename T_TYPE::SPEC::HIDDEN_LAYER>()";
            }
            ss_initializer_list_create << "}, ";
            ss_initializer_list_create_function << "}, ";
            ss_initializer_list_create << "output_layer::factory<typename T_TYPE::SPEC::OUTPUT_LAYER>}";
            ss_initializer_list_create_function << "output_layer::factory_function<typename T_TYPE::SPEC::OUTPUT_LAYER>()";
            ss_initializer_list_create << ", log_std::parameters}";
            ss_initializer_list_create_function << ", log_std::parameters}";
        }

        std::string initializer_list, initializer_list_create;
        if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Forward){
            initializer_list = ss_initializer_list.str();
            initializer_list_create = ss_initializer_list_create.str();
        }
        else{
            if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Backward){
                initializer_list = "{" + ss_initializer_list.str() + "}";
                initializer_list_create = "{" + ss_initializer_list_create.str() + "}";
            }
            else{
                if constexpr(SPEC::CAPABILITY::TAG == nn::LayerCapability::Gradient){
                    initializer_list = "{{" + ss_initializer_list.str() + "}}";
                    initializer_list_create = "{{" + ss_initializer_list_create.str() + "}}";
                }
                else{
                    utils::assert_exit(device, false, "Unknown capability");
                }
            }
        }
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "TYPE module = " << initializer_list << ";\n";
        ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory = " << initializer_list_create << ";" << "\n";
        ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory_function(){return T_TYPE" << initializer_list_create << ";" << "}\n";
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template<typename DEVICE, typename SPEC, template <typename> typename BASE>
    persist::Code save_code_split(DEVICE& device, nn_models::mlp_unconditional_stddev::NeuralNetworkBackward<SPEC, BASE>& network, std::string name, bool const_declaration=true, typename DEVICE::index_t indent = 0){
        return save_code_split(device, static_cast<nn_models::mlp_unconditional_stddev::NeuralNetworkForward<SPEC, BASE>&>(network), name, const_declaration, indent);
    }
    template<typename DEVICE, typename SPEC, template <typename> typename BASE>
    persist::Code save_code_split(DEVICE& device, nn_models::mlp_unconditional_stddev::NeuralNetworkGradient<SPEC, BASE>& network, std::string name, bool const_declaration=true, typename DEVICE::index_t indent = 0){
        return save_code_split(device, static_cast<nn_models::mlp_unconditional_stddev::NeuralNetworkBackward<SPEC, BASE>&>(network), name, const_declaration, indent);
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn_models::mlp_unconditional_stddev::NeuralNetworkForward<SPEC>& network, std::string name, bool const_declaration = true, typename DEVICE::index_t indent = 0) {
        auto code = save_code_split(device, network, name, const_declaration, indent);
        return code.header + code.body;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
