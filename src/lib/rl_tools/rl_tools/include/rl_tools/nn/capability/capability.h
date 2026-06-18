#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_CAPABILITY_CAPABILITY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_CAPABILITY_CAPABILITY_H
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

#include "../parameters/parameters.h"


// The Capability Configuraiton carries properties that apply to multiple parts of a model (e.g. the high-level mode, the layers, etc.)
// This allows e.g. to simply change the capabilities of a model without adapting all the involved specifications individually

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn{
    enum class LayerCapability{
        Forward, // just forward
        Backward, // forward + backward wrt to the input
        Gradient // forward + backward wrt to the input + backward wrt to the weights
    };
    /*
     Tags that can also carry the parameter type.
     The LayerCapability determines some fields in the layers (e.g. the intermediate outputs are required for gradient calculations, while for the backward pass wrt. the input only the pre-activations might be needed)
     The PARAMETER_TYPE determines the type of the parameters (e.g. adam requries first and second order moments in addition to the gradient)
     This Capability system allows the switching of models for e.g. checkpointing: We are training a full model with gradients, and optimizers state then convert it to a forward only model (just the parameters) and save it as a checkpoint.
    */
    namespace capability{
        template <bool T_DYNAMIC_ALLOCATION=true, bool T_CONST=false>
        struct Forward{
            static constexpr LayerCapability TAG = LayerCapability::Forward;
            using PARAMETER_TYPE = nn::parameters::Plain;
            static constexpr auto BATCH_SIZE = 1;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static constexpr bool CONST = T_CONST;
            template <bool TT_DYNAMIC_ALLOCATION=true, bool TT_CONST=false>
            using CHANGE_PARAMETERS = Forward<TT_DYNAMIC_ALLOCATION, TT_CONST>;
        };
        template <bool T_DYNAMIC_ALLOCATION=true, bool T_CONST=false>
        struct Backward{
            static constexpr LayerCapability TAG = LayerCapability::Backward;
            using PARAMETER_TYPE = nn::parameters::Plain;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static constexpr bool CONST = T_CONST;
            template <bool TT_DYNAMIC_ALLOCATION=true, bool TT_CONST=false>
            using CHANGE_PARAMETERS = Backward<TT_DYNAMIC_ALLOCATION, TT_CONST>;
        };
        template <typename T_PARAMETER_TYPE, bool T_DYNAMIC_ALLOCATION=true, bool T_CONST=false>
        struct Gradient{
            static constexpr LayerCapability TAG = LayerCapability::Gradient;
            using PARAMETER_TYPE = T_PARAMETER_TYPE;
            static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
            static_assert(!utils::typing::is_same_v<T_PARAMETER_TYPE, nn::parameters::Plain>);
            static constexpr bool CONST = T_CONST;
            template <bool TT_DYNAMIC_ALLOCATION=true, bool TT_CONST=false>
            using CHANGE_PARAMETERS = Gradient<PARAMETER_TYPE, TT_DYNAMIC_ALLOCATION, TT_CONST>;
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif

