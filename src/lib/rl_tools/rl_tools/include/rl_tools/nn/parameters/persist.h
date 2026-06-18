#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_PARAMETERS_PERSIST_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_PARAMETERS_PERSIST_H

#include "../../nn/parameters/parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::parameters::Plain::Instance<SPEC>& parameter, GROUP& group) {
        save(device, parameter.parameters, group, "parameters");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, nn::parameters::Gradient::Instance<SPEC>& parameter, GROUP& group) {
        save(device, (nn::parameters::Plain::Instance<SPEC>&)parameter, group);
        save(device, parameter.gradient, group, "gradient");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::parameters::Plain::Instance<SPEC>& parameter, GROUP& group) {
        return load(device, parameter.parameters, group, "parameters");
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, nn::parameters::Gradient::Instance<SPEC>& parameter, GROUP& group) {
        bool success = load(device, (nn::parameters::Plain::Instance<SPEC>&)parameter, group);
        success &= load(device, parameter.gradient, group, "gradient");
        return success;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
