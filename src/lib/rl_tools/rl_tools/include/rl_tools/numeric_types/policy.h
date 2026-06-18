#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NUMERIC_TYPES_POLICY_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NUMERIC_TYPES_POLICY_H

#include "bf16.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::numeric_types{
    template<typename T_TAG, typename T_TYPE>
    struct UseCase{
        using TAG = T_TAG;
        using TYPE = T_TYPE;
    };

    template<typename TAG, typename DEFAULT, typename... USE_CASES>
    struct LookupType {
        using T = DEFAULT;
    };

    template<typename TAG, typename DEFAULT, typename FIRST_USE_CASE, typename... REST_USE_CASES>
    struct LookupType<TAG, DEFAULT, FIRST_USE_CASE, REST_USE_CASES...> {
        using T = utils::typing::conditional_t<
            utils::typing::is_same_v<typename FIRST_USE_CASE::TAG, TAG>,
            typename FIRST_USE_CASE::TYPE,
            typename LookupType<TAG, DEFAULT, REST_USE_CASES...>::T
        >;
    };
    template<typename T_DEFAULT, typename... USE_CASES>
    struct Policy{
        using DEFAULT = T_DEFAULT;
        template<typename TAG>
        using GET = typename LookupType<TAG, DEFAULT, USE_CASES...>::T;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
