#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_REGISTRY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_REGISTRY_H

#include "dynamics/crazyflie.h"
#include "dynamics/mrs.h"
#include "dynamics/arpl.h"
#include "dynamics/x500_real.h"
#include "dynamics/x500_sim.h"
#include "dynamics/fs.h"
#include "dynamics/flightmare.h"
#include "dynamics/soft.h"
#include "dynamics/soft_rigid.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters{
    namespace dynamics{
        enum class REGISTRY{
            crazyflie,
            mrs,
            x500_real,
            x500_sim,
            arpl,
            fs_base,
            flightmare,
            soft,
            soft_rigid,
        };
        template <REGISTRY MODEL, typename SPEC>
        constexpr auto registry = [](){
            if constexpr (MODEL == REGISTRY::crazyflie){
                return dynamics::crazyflie<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::mrs){
                return dynamics::mrs<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::x500_real){
                return dynamics::x500::real<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::x500_sim){
                return dynamics::x500::sim<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::arpl){
                return dynamics::arpl<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::fs_base){
                return dynamics::fs::base<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::flightmare){
                return dynamics::flightmare<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::soft){
                return dynamics::soft<typename SPEC::T, typename SPEC::TI>;
            }else if constexpr (MODEL == REGISTRY::soft_rigid){
                return dynamics::soft_rigid<typename SPEC::T, typename SPEC::TI>;
            }else{
                static_assert(rl_tools::utils::typing::dependent_false<SPEC>, "Unknown model");
            }
        }();

        template <auto THING>
        struct Dependent{};

        template <REGISTRY MODEL>
        constexpr auto registry_name = [](){
            if constexpr (MODEL == REGISTRY::crazyflie){
                return "crazyflie";
            }else if constexpr (MODEL == REGISTRY::mrs){
                return "mrs";
            }else if constexpr (MODEL == REGISTRY::x500_real){
                return "x500_real";
            }else if constexpr (MODEL == REGISTRY::x500_sim){
                return "x500_sim";
            }else if constexpr (MODEL == REGISTRY::fs_base){
                return "fs_base";
            }else if constexpr (MODEL == REGISTRY::flightmare){
                return "flightmare";
            }else if constexpr (MODEL == REGISTRY::soft){
                return "soft";
            }else if constexpr (MODEL == REGISTRY::soft_rigid){
                return "soft_rigid";
            }else{
                static_assert(rl_tools::utils::typing::dependent_false<Dependent<MODEL>>, "Unknown model");
            }
        }();
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
