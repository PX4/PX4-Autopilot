#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_OPTIMIZERS_ADAM_INSTANCE_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_OPTIMIZERS_ADAM_INSTANCE_PERSIST_CODE_H

#include "../adam.h"
#include "../../../../nn/parameters/persist_code.h"

#include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    std::string get_type_string(nn::parameters::Adam p){
        return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Adam";
    }
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, nn::parameters::Adam::Instance<SPEC>& parameter, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0, bool output_memory_only=false){
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss, ss_header;
        auto plain = save_code_split(device, (nn::parameters::Gradient::Instance<SPEC>&) parameter, name, const_declaration, indent, true);
        ss_header << plain.header;
        ss_header << "#include <rl_tools/nn/optimizers/adam/adam.h>\n";
        ss << plain.body;
        ss << ind << "namespace " << name << " {\n";
        auto gradient_first_order_moment = save_code_split(device, parameter.gradient_first_order_moment, "gradient_first_order_moment_memory", const_declaration, indent+1);
        ss_header << gradient_first_order_moment.header;
        ss << gradient_first_order_moment.body;
        auto gradient_second_order_moment = save_code_split(device, parameter.gradient_second_order_moment, "gradient_second_order_moment_memory", const_declaration, indent+1);
        ss_header << gradient_second_order_moment.header;
        ss << gradient_second_order_moment.body;
        if(!output_memory_only){
            // ss << ind << "    " << "static_assert(RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::utils::typing::is_same_v<parameters_memory::CONTAINER_TYPE, gradient_memory::CONTAINER_TYPE>);\n";
            // ss << ind << "    " << "static_assert(RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::utils::typing::is_same_v<gradient_memory::CONTAINER_TYPE, gradient_first_order_moment_memory::CONTAINER_TYPE>);\n";
            // ss << ind << "    " << "static_assert(RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::utils::typing::is_same_v<gradient_memory::CONTAINER_TYPE, gradient_second_order_moment_memory::CONTAINER_TYPE>);\n";
            // ss << ind << "    " << "using PARAMETER_SPEC = " << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Adam::Specification<parameters_memory::CONTAINER_TYPE, "
            ss << ind << "    " << "using TYPE_POLICY = " << to_string(typename SPEC::TYPE_POLICY{}) << ";\n";
            ss << ind << "    " << "using PARAMETER_SPEC = " << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Adam::Specification<TYPE_POLICY, typename parameters_memory::SPEC::TI, typename parameters_memory::SPEC::SHAPE, "
               << get_type_string_tag(device, typename SPEC::GROUP_TAG{})
               << ", "
               << get_type_string_tag(device, typename SPEC::CATEGORY_TAG{})
               << ", true, true>;\n";
            ss << ind << "    " << (const_declaration ? "constexpr " : "") << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::parameters::Adam::Instance<PARAMETER_SPEC> parameters = {{{parameters_memory::container}, gradient_memory::container}, gradient_first_order_moment_memory::container, gradient_second_order_moment_memory::container};\n";
        }
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, nn::parameters::Adam::Instance<SPEC>& p) {
        std::string data;
        data += "{";
        data += nn_analytics(device, static_cast<nn::parameters::Gradient::Instance<SPEC>&>(p), true) + ", ";
        data += "\"gradient_first_order_moment\": " + json(device, p.gradient_first_order_moment) + ", ";
        data += "\"gradient_second_order_moment\": " + json(device, p.gradient_second_order_moment);
        data += "}";
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif
