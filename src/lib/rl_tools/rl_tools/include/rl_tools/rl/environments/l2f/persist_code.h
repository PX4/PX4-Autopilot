#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PERSIST_CODE_H

#include "../../../persist/code.h"
#include "../../../utils/generic/typing.h"
#include <sstream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, rl::environments::Multirotor<SPEC>& env, typename rl::environments::Multirotor<SPEC>::Parameters& parameters, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss_header;
        ss_header << "#include <rl_tools/rl/environments/l2f/multirotor.h>\n";
        std::stringstream ss;
        ss << ind << "namespace " << name << " {\n";
        ss << ind << "    using T = " << containers::persist::get_type_string<T>() << ";\n";
        ss << ind << "    using TI = " << containers::persist::get_type_string<TI>() << ";\n";
        ss << ind << "    namespace parameters{\n";
        ss << ind << "        namespace mdp{\n";
        ss << ind << "            namespace init{\n";
        ss << ind << "                static constexpr T max_position = " << parameters.mdp.init.max_position << ";\n";;
        ss << ind << "                static constexpr T max_angle = " << parameters.mdp.init.max_angle << ";\n";;
        ss << ind << "                static constexpr T max_linear_velocity = " << parameters.mdp.init.max_linear_velocity << ";\n";;
        ss << ind << "                static constexpr T max_angular_velocity = " << parameters.mdp.init.max_angular_velocity << ";\n";;
        ss << ind << "            }\n";
        ss << ind << "            namespace termination{\n";
        ss << ind << "                static constexpr T position_threshold = " << parameters.mdp.termination.position_threshold << ";\n";;
        ss << ind << "                static constexpr T linear_velocity_threshold = " << parameters.mdp.termination.linear_velocity_threshold << ";\n";;
        ss << ind << "                static constexpr T angular_velocity_threshold = " << parameters.mdp.termination.angular_velocity_threshold << ";\n";;
        ss << ind << "            }\n";
        ss << ind << "        }\n";
        ss << ind << "    }\n";
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code_env(DEVICE& device, rl::environments::Multirotor<SPEC>& env, typename rl::environments::Multirotor<SPEC>::Parameters& parameters, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        auto code = save_code_split(device, env, parameters, name, const_declaration, indent);
        return code.header + code.body;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
