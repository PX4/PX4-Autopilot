#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_ENVIRONMENTS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_ENVIRONMENTS_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_T, typename T_TI>
    struct Environment{
        static constexpr T_TI N_AGENTS = 1;
    };

    template <typename ENV>
    struct PREVENT_DEFAULT : rl_tools::utils::typing::false_type {};
    template <typename ENV>
    struct PREVENT_DEFAULT_GET_UI : rl_tools::utils::typing::conditional_t<PREVENT_DEFAULT<ENV>::value, rl_tools::utils::typing::true_type, rl_tools::utils::typing::false_type> {};
    template <typename ENV>
    struct PREVENT_DEFAULT_GET_DESCRIPTION : rl_tools::utils::typing::conditional_t<PREVENT_DEFAULT<ENV>::value, rl_tools::utils::typing::true_type, rl_tools::utils::typing::false_type> {};
    struct DummyUI{};
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#include "acrobot/acrobot.h"
#include "car/car.h"
//#include "mujoco/mujoco.h" // mujoco is not generic so needs to be included separately
#include "pendulum/pendulum.h"

#endif