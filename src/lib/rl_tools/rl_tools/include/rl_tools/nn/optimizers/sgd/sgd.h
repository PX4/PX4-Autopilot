#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_OPTIMIZERS_SGD_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_OPTIMIZERS_SGD_H

#include "../../../nn/parameters/parameters.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::optimizers::sgd {
    template<typename T>
    struct DefaultParameters{
    public:
        static constexpr T ALPHA = 0.001;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::parameters{
    struct SGD{
        template <typename T_SPEC>
        struct instance: Gradient::Instance<T_SPEC>{};
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
