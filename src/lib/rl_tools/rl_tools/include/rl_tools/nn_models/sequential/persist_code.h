#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_SEQUENTIAL_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_SEQUENTIAL_PERSIST_CODE_H
#include "../../containers/matrix/persist_code.h"
#include "../../persist/code.h"
#include "../../nn/persist_code.h"
#include "model.h"

#include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn_models::sequential::ModuleForward<SPEC>& model, std::string name, bool const_declaration=true, typename DEVICE::index_t indent = 0, typename DEVICE::index_t layer_i = 0) {
        // using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss, ss_header;
        auto layer_output = save_code_split(device, model.content, "layer_" + std::to_string(layer_i), const_declaration, indent+1);
        ss_header << layer_output.header;
        ss_header << "#include <rl_tools/nn_models/sequential/model.h>\n";
        if(layer_i == 0){
            ss << ind << "namespace " << name << " {\n";
        }
        ss << layer_output.body;
        if constexpr(!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            auto downstream_output = save_code_split(device, model.next_module, name, const_declaration, indent, layer_i+1);
            ss_header << downstream_output.header;
            ss << downstream_output.body;
        }
        if(layer_i == 0){
            ss << ind << "    " << "namespace model_definition {\n";
//            ss << ind << "    " << "    " << "using namespace RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::sequential::interface;\n";
//            std::string capability = "Forward";
            ss << ind << "    " << "    " << "using CAPABILITY = " << to_string(typename SPEC::CAPABILITY::template CHANGE_PARAMETERS<true, true>{}) << "; \n";
            ss << ind << "    " << "    " << "template <typename T_CONTENT, typename T_NEXT_MODULE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::sequential::OutputModule>\n";
            ss << ind << "    " << "    " << "using Module = typename RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::sequential::Module<T_CONTENT, T_NEXT_MODULE>;\n";
            ss << ind << "    " << "    " << "using MODULE_CHAIN = Module<";
            for(TI layer_i = 0; layer_i < num_layers(model); layer_i++){
                ss << "layer_" << layer_i << "::TEMPLATE";
                if(layer_i < num_layers(model)-1){
                    ss << ", Module<";
                }
            }
            for(TI layer_i = 0; layer_i < num_layers(model); layer_i++){
                ss << ">";
            }
            ss << ";\n";
            ss << ind << "    " << "    " << "using MODEL = typename RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, layer_0::INPUT_SHAPE>;\n";
            ss << ind << "    " << "}\n";
            ss << ind << "    " << "using TYPE = model_definition::MODEL;\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "TYPE module = {";
            std::string model_stub = "TYPE"; // this is required because we can not instantiate layers before defining the MODEL, as the model dictates the layer types through the INPUT_SHAPE mangling process
            std::stringstream ss_initializer_list;
            for(TI inner_layer_i = 0; inner_layer_i < num_layers(model); inner_layer_i++){
                ss_initializer_list << "layer_" << inner_layer_i << "::factory<" << model_stub << "::CONTENT>";
                if(inner_layer_i < num_layers(model)-1){
                    ss_initializer_list << ", {";
                }
                model_stub += "::NEXT_MODULE";
            }
            ss_initializer_list << ", {}";
            for(TI inner_layer_i = 0; inner_layer_i < num_layers(model); inner_layer_i++){
                ss_initializer_list << "}";
            }
            ss << ss_initializer_list.str() << ";\n";

            std::stringstream ss_initializer_list_create, ss_initializer_list_create_function;
            std::string model_stub_create = "T_TYPE"; // this is required because we can not instantiate layers before defining the MODEL, as the model dictates the layer types through the INPUT_SHAPE mangling process
            for(TI inner_layer_i = 0; inner_layer_i < num_layers(model); inner_layer_i++){
                ss_initializer_list_create << "layer_" << inner_layer_i << "::factory<typename " << model_stub_create << "::CONTENT>";
                ss_initializer_list_create_function << "layer_" << inner_layer_i << "::factory_function<typename " << model_stub_create << "::CONTENT>()";
                if(inner_layer_i < num_layers(model)-1){
                    ss_initializer_list_create << ", {";
                    ss_initializer_list_create_function << ", {";
                }
                model_stub_create += "::NEXT_MODULE";
            }
            ss_initializer_list_create << ", {}";
            ss_initializer_list_create_function << ", {}";
            for(TI inner_layer_i = 0; inner_layer_i < num_layers(model); inner_layer_i++){
                ss_initializer_list_create << "}";
                ss_initializer_list_create_function << "}";
            }
            std::string initializer_list = ss_initializer_list_create.str();
            ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory = {" << initializer_list << ";" << "\n";
            ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory_function(){return T_TYPE{" << ss_initializer_list_create_function.str() << ";" << "}\n";
            ss << ind << "}";


//            ss << ind << "    " << (const_declaration ? "const " : "") << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::sequential::Module<" << layer_i << "> module = {layer_0::container, " << get_type_string<typename SPEC::NEXT_MODULE>() << "::module, };\n";
        }
        return {ss_header.str(), ss.str()};
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn_models::sequential::ModuleForward<SPEC>& network, std::string name, bool const_declaration = true, typename DEVICE::index_t indent = 0) {
        auto code = save_code_split(device, network, name, const_declaration, indent);
        return code.header + code.body;
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn_models::sequential::ModuleGradient<SPEC>& model, typename DEVICE::index_t layer_i = 0) {
        std::string data;
        if(layer_i == 0){
            data += "{\"layers\":[";
        }
        data += nn_analytics(device, model.content);
        if constexpr (!utils::typing::is_same_v<typename SPEC::NEXT_MODULE, nn_models::sequential::OutputModule>){
            data += ", ";
            data += nn_analytics(device, model.next_module, layer_i + 1);
        }
        if(layer_i == 0){
            data += "]}";
        }
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
