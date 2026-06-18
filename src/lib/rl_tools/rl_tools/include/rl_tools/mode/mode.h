#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_MODE_MODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_MODE_MODE_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    // note: please always check for the mode by using utils::typing::is_base_of_v, e.g. `utils::typing::is_base_of_v<mode::Evaluation, MODE>`. This ensures that when some layers of e.g. an nn_models::Sequential model are using specific modes that there are no side-effects
    // note: please use the same mode for affiliated forward and backward passes
    template <typename T_MODE>
    struct Mode: T_MODE{
        using MODE = T_MODE;
    };
    namespace mode{
        struct Final{ };
        template <typename T_BASE = Final, typename T_SPEC = bool>
        struct Default: Final{
            using SPEC = bool;
            using BASE = Final;
        };
        template <typename T_BASE = Final, typename T_SPEC = bool>
        struct Rollout: T_BASE{
            // this is what is passed by on/off-policy runners
            // semantics: randomness for exploration
            using SPEC = T_SPEC;
            using BASE = T_BASE;
        };
        template <typename T_BASE = Final, typename T_SPEC = bool>
        struct Evaluation: T_BASE{
            // this is what is passed by rl::utils::evaluation
            // semantics: no randomness
            using SPEC = T_SPEC;
            using BASE = T_BASE;
        };
        template <typename MODE>
        constexpr bool _check_carrier(MODE){
            return false;
        }
        template <typename MODE>
        constexpr bool _check_carrier(Mode<MODE>){
            return true;
        }

        template <typename INPUT, template <typename, typename> typename MODE>
        constexpr bool _is(){
            static_assert(!_check_carrier(INPUT{}), "You should only check the unwrapped mode => execute mode::is<MODE> if you have a Mode<MODE>");
            if constexpr (utils::typing::is_same_v<typename INPUT::BASE, Final>){
                return utils::typing::is_same_v<INPUT, MODE<Final, bool>>;
            }
            else{
                return utils::typing::is_same_v<MODE<typename INPUT::BASE, typename INPUT::SPEC>, INPUT> || _is<typename INPUT::BASE, MODE>();
            }
        }

        template <typename INPUT, template <typename, typename> typename MODE>
        constexpr bool is = _is<INPUT, MODE>();

        namespace sequential{
            template <typename T_RESET_CONTAINER_TYPE>
            struct ResetMaskSpecification{
                using RESET_CONTAINER_TYPE = T_RESET_CONTAINER_TYPE;
            };
            template <typename T_BASE, typename T_SPEC>
            struct ResetMask: T_BASE{
                using SPEC = T_SPEC;
                using BASE = T_BASE;
                typename SPEC::RESET_CONTAINER_TYPE mask;
            };

        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
