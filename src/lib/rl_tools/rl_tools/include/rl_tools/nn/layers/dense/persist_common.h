
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_DENSE_PERSIST_COMMON_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_DENSE_PERSIST_COMMON_H
#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace nn::layers::dense::persist{
        template<nn::activation_functions::ActivationFunction ACTIVATION_FUNCTION>
        auto get_activation_function_string_short(){
            static_assert(ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::IDENTITY ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::RELU ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::GELU ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::TANH ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::FAST_TANH ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::SIGMOID);

            if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::IDENTITY){
                return "IDENTITY";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::RELU){
                return "RELU";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::TANH){
                return "TANH";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::FAST_TANH){
                return "FAST_TANH";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::SIGMOID){
                return "SIGMOID";
            }
        }
        template<nn::activation_functions::ActivationFunction ACTIVATION_FUNCTION>
        constexpr auto get_activation_function_string(){
            static_assert(ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::IDENTITY ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::RELU ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::GELU ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::TANH ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::FAST_TANH ||
                          ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::SIGMOID);

            if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::IDENTITY){
                return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::activation_functions::ActivationFunction::IDENTITY";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::RELU){
                return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::activation_functions::ActivationFunction::RELU";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::TANH){
                return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::activation_functions::ActivationFunction::TANH";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::FAST_TANH){
                return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::activation_functions::ActivationFunction::FAST_TANH";
            } else if constexpr (ACTIVATION_FUNCTION == nn::activation_functions::ActivationFunction::SIGMOID){
                return "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::nn::activation_functions::ActivationFunction::SIGMOID";
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
