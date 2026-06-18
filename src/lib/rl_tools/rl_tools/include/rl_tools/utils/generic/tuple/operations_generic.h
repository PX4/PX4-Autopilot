#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_GENERIC_TUPLE_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_GENERIC_TUPLE_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename DEVICE, typename T_TI>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, utils::Tuple<T_TI>& tuple){ }
    template <typename DEVICE, typename T_TI, typename CURRENT_TYPE, typename... Types>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, utils::Tuple<T_TI, CURRENT_TYPE, Types...>& tuple){
        malloc(device, tuple.content);
        if constexpr(sizeof...(Types) > 0){
            malloc(device, static_cast<utils::Tuple<T_TI, Types...>&>(tuple));
        }
    }
    template <typename DEVICE, typename T_TI, template <typename> typename F>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, utils::MapTuple<utils::Tuple<T_TI>, F>& tuple){ }
    template <typename DEVICE, typename T_TI, template <typename> typename F, typename CURRENT_TYPE, typename... Types>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, utils::MapTuple<utils::Tuple<T_TI, CURRENT_TYPE, Types...>, F>& tuple){
        malloc(device, tuple.content);
        if constexpr(sizeof...(Types) > 0){
            malloc(device, static_cast<utils::MapTuple<utils::Tuple<T_TI, Types...>, F>&>(tuple));
        }
    }

    template <typename DEVICE, typename T_TI>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, utils::Tuple<T_TI>& tuple){ }
    template <typename DEVICE, typename T_TI, typename CURRENT_TYPE, typename... Types>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, utils::Tuple<T_TI, CURRENT_TYPE, Types...>& tuple){
        free(device, tuple.content);
        if constexpr(sizeof...(Types) > 0){
            free(device, static_cast<utils::Tuple<T_TI, Types...>&>(tuple));
        }
    }
    template <typename DEVICE, typename T_TI, template <typename> typename F>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, utils::MapTuple<utils::Tuple<T_TI>, F>& tuple){ }
    template <typename DEVICE, typename T_TI, template <typename> typename F, typename CURRENT_TYPE, typename... Types>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, utils::MapTuple<utils::Tuple<T_TI, CURRENT_TYPE, Types...>, F>& tuple){
        free(device, tuple.content);
        if constexpr(sizeof...(Types) > 0){
            free(device, static_cast<utils::MapTuple<utils::Tuple<T_TI, Types...>, F>&>(tuple));
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif