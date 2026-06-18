#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_ACTIVATION_FUNCTIONS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_ACTIVATION_FUNCTIONS_H
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif
#include "../devices/devices.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn::activation_functions {
    enum ActivationFunction{
        IDENTITY,
        RELU,
        GELU,
        TANH,
        FAST_TANH,
        SIGMOID
    };
    template<enum ActivationFunction F>
    constexpr bool check_activation_function = F == IDENTITY || F == RELU || F == GELU || F == TANH || F == FAST_TANH || F == SIGMOID;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename T, nn::activation_functions::ActivationFunction F>
    RL_TOOLS_FUNCTION_PLACEMENT inline T activation(T x){
        using namespace nn::activation_functions;
        static_assert(DEVICE::TYPE == devices::Type::math, "DEVICE is not a math device");
        static_assert(nn::activation_functions::check_activation_function<F>, "Invalid activation function");
        if constexpr(F == IDENTITY){
            return x;
        }
        else if constexpr(F == RELU){
            return math::max(DEVICE(), x, (T)0);
        }
        else if constexpr(F == GELU){
            constexpr T a = math::FRAC_2_SQRTPI<T> * math::SQRT1_2<T> * (T)0.5;
            return (T)0.5 * (x + x * math::tanh(DEVICE(), a * ((T)0.044715f * x * x * x + x)));
        }
        else if constexpr(F == TANH){
            return math::tanh(DEVICE(), x);
        }
        else if constexpr(F == FAST_TANH){
            return math::fast_tanh(DEVICE(), x);
        }
        else if constexpr(F == SIGMOID){
            return (T)1 / ((T)1 + math::exp(DEVICE(), -x));
        }
        else{
            return 0;
        }
    }

    template<typename DEVICE, typename T, nn::activation_functions::ActivationFunction F>
    RL_TOOLS_FUNCTION_PLACEMENT inline T d_activation_d_x(T x){
        using namespace nn::activation_functions;
        static_assert(DEVICE::TYPE == devices::Type::math, "DEVICE is not a math device");
        static_assert(check_activation_function<F>, "Invalid activation function");
        if constexpr(F == IDENTITY){
            return 1;
        }
        else if constexpr(F == RELU){
            return x > 0 ? 1 : 0;
        }
        else if constexpr(F == GELU){
            constexpr T a = math::FRAC_2_SQRTPI<T> * math::SQRT1_2<T> * (T)0.5;
            constexpr T b = 0.044715f;
            T tanh_term = math::tanh(DEVICE(), a * (b * x * x * x + x));
            return (T)0.5*((T)1 + tanh_term) + (T)0.5 * x * ((T)1 - tanh_term * tanh_term) * a * ((T)3 * b * x * x + (T)1);
        }
        else if constexpr(F == TANH){
            T a = math::tanh(DEVICE(), x);
            return (T)1 - a * a;
        }
        else if constexpr(F == FAST_TANH){
            T a = math::fast_tanh(DEVICE(), x);
            return (T)1 - a * a;
        }
        else if constexpr(F == SIGMOID){
            return activation<DEVICE, T, SIGMOID>(x) * (1 - activation<DEVICE, T, SIGMOID>(x));
        }
        else{
            return 0;
        }
    }
    template<typename DEVICE, typename T, nn::activation_functions::ActivationFunction F>
    RL_TOOLS_FUNCTION_PLACEMENT auto name(){
        using namespace nn::activation_functions;
        if constexpr(F == IDENTITY){
            return "IDENTITY";
        }
        else if constexpr(F == RELU){
            return "RELU";
        }
        else if constexpr(F == GELU){
            return "GELU";
        }
        else if constexpr(F == TANH){
            return "TANH";
        }
        else if constexpr(F == FAST_TANH){
            return "FAST_TANH";
        }
        else if constexpr(F == SIGMOID){
            return "SIGMOID";
        }
        else{
            return "NIL";
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif