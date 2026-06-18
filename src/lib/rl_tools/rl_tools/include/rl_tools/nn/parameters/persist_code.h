#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_PARAMETERS_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_PARAMETERS_PERSIST_CODE_H
#include "../../containers/matrix/persist_code.h"
#include "parameters.h"
#include <sstream>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {

    std::string get_type_string(nn::parameters::Plain p){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Plain";
    }
    std::string get_type_string(nn::parameters::Gradient p){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Gradient";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::categories::Weights){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::categories::Weights";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::categories::Biases){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::categories::Biases";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::categories::Constant){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::categories::Constant";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::groups::Normal){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::groups::Normal";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::groups::Input){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::groups::Input";
    }

    template <typename DEVICE>
    std::string get_type_string_tag(const DEVICE&, const nn::parameters::groups::Output){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::groups::Output";
    }

    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::parameters::Plain::Instance<SPEC>& parameter, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0, bool output_memory_only=false){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss, ss_header;
        ss << ind << "namespace " << name << " {\n";
        auto container = save_code_split(device, parameter.parameters, "parameters_memory", const_declaration, indent+1);
        ss_header << container.header;
        ss_header << "#include <rl_tools/nn/parameters/parameters.h>\n";
        ss << container.body;
        if(!output_memory_only){
            ss << ind << "    " << "using TYPE_POLICY = " << to_string(typename SPEC::TYPE_POLICY{}) << ";\n";
            ss << ind << "    " << "using PARAMETER_SPEC = " << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Plain::Specification<TYPE_POLICY, typename parameters_memory::SPEC::TI, typename parameters_memory::SPEC::SHAPE, "
            << get_type_string_tag(device, typename SPEC::GROUP_TAG{})
            << ", "
            << get_type_string_tag(device, typename SPEC::CATEGORY_TAG{})
            << ", true, true>;\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Plain::Instance<PARAMETER_SPEC> parameters = {parameters_memory::container};\n";
        }
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }

    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::parameters::Gradient::Instance<SPEC>& parameter, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0, bool output_memory_only=false){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss, ss_header;
        ss_header << "#include <rl_tools/utils/generic/typing.h>\n";
        auto plain = save_code_split(device, (nn::parameters::Plain::Instance<SPEC>&) parameter, name, const_declaration, indent, true);
        ss_header << plain.header;
        ss << plain.body;
        ss << ind << "namespace " << name << " {\n";
        auto gradient = save_code_split(device, parameter.gradient, "gradient_memory", const_declaration, indent+1);
        ss_header << gradient.header;
        ss << gradient.body;
        if(!output_memory_only){
            ss << ind << "    " << "using TYPE_POLICY = " << to_string(typename SPEC::TYPE_POLICY{}) << ";\n";
            ss << ind << "    " << "using PARAMETER_SPEC = " << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Gradient::Specification<TYPE_POLICY, typename parameters_memory::SPEC::TI, typename parameters_memory::SPEC::SHAPE, "
            << get_type_string_tag(device, typename SPEC::GROUP_TAG{})
            << ", "
            << get_type_string_tag(device, typename SPEC::CATEGORY_TAG{})
            << ", true, true>;\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Gradient::Instance<PARAMETER_SPEC> parameters = {parameters_memory::container};\n";
        }
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn::parameters::Plain::Instance<SPEC>& p, bool downstream=false) {
        std::string data;
        if (!downstream){
            data += "{";
        }
        data += "\"parameters\": " + json(device, p.parameters);
        if (!downstream){
            data += "}";
        }
        return data;
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn::parameters::Gradient::Instance<SPEC>& p, bool downstream=false) {
        std::string data;
        if (!downstream){
            data += "{";
        }
        data += nn_analytics(device, static_cast<nn::parameters::Plain::Instance<SPEC>&>(p), true) + ", ";
        data += "\"gradient\": " + json(device, p.gradient);
        if (!downstream){
            data += "}";
        }
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif