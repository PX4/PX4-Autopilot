#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_MODELS_MULTI_AGENT_WRAPPER_PERSIST_CODE_H



#include <string>
#include <sstream>
#include "../../persist/code.h"
#include "model.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<SPEC>& module, std::string name, bool const_declaration=true, typename DEVICE::index_t indent = 0){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss_header;
        ss_header << "#include <rl_tools/nn_models/multi_agent_wrapper/model.h>\n";
        std::stringstream ss;
        ss << ind << "namespace " << name << " {\n";
        auto content = save_code_split(device, module.content, "content", const_declaration, indent+1);
        ss_header << content.header;
        ss << content.body;
        std::string T_string = containers::persist::get_type_string<typename SPEC::TYPE_POLICY::DEFAULT>();
        std::string TI_string = containers::persist::get_type_string<TI>();
        ss << "\n";
        ss << ind << "    using TYPE_POLICY = " << to_string(typename SPEC::TYPE_POLICY{}) << ";\n";
        ss << ind << "    using CONFIG = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::multi_agent_wrapper::Configuration<";
        ss << "TYPE_POLICY, ";
        ss << TI_string << ", ";
        ss << std::to_string(SPEC::N_AGENTS) << ", ";
        ss << "content::model_definition::MODULE_CHAIN";
        ss << ">; \n";
        ss << ind << "    using CAPABILITY = " << to_string(typename SPEC::CAPABILITY::template CHANGE_PARAMETERS<true, true>{}) << "; \n";
        ss << ind << "    using INPUT_SHAPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::Shape<" << TI_string << ", " << SPEC::INPUT_SHAPE::template GET<0> << ", " << SPEC::INPUT_SHAPE::template GET<1> << ", " << SPEC::INPUT_SHAPE::template GET<2> << ">;\n";
        ss << ind << "    using TYPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn_models::multi_agent_wrapper::Module<CONFIG, CAPABILITY, INPUT_SHAPE>; \n";
        std::stringstream ss_initializer_list, ss_initializer_list_function;
        ss_initializer_list << "{content::factory<TYPE::MODEL>}";
        ss_initializer_list_function << "{content::factory_function<TYPE::MODEL>()}";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "TYPE module = " << ss_initializer_list.str() << ";\n";
        ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory = " << ss_initializer_list.str() << ";" << "\n";
        ss << ind << "    " << "template <typename T_TYPE = TYPE>" << "\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "T_TYPE factory_function(){return " << ss_initializer_list_function.str() << ";}\n";
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, nn_models::multi_agent_wrapper::ModuleForward<SPEC>& module, std::string name, bool const_declaration = true, typename DEVICE::index_t indent = 0) {
        auto code = save_code_split(device, module, name, const_declaration, indent);
        return code.header + code.body;
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn_models::multi_agent_wrapper::ModuleGradient<SPEC>& nn) {
        std::string data;
        data += "{";
        data += "   \"type\": \"multi_agent_wrapper\",";
        data += "   \"inner\": ";
        data += nn_analytics(device, nn.content);
        data += "}";
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
