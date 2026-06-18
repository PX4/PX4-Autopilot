#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_TENSOR_OPERATIONS_CPU_H

#include "tensor.h"
#include "operations_generic.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEV_SPEC, typename SPEC>
    void print(devices::CPU<DEV_SPEC>& device, const Tensor<SPEC>& tensor, bool python_literal=false, typename DEV_SPEC::index_t level=0){
        using TI = typename DEV_SPEC::index_t;
        using T = typename SPEC::T;
        if constexpr(length(typename SPEC::SHAPE{}) == 1){
            std::cout << (python_literal ? "[" : "");
            for(TI i=0; i < get<0>(typename SPEC::SHAPE{}); i++){
                std::cout << get(device, tensor, i) << " " << (python_literal ? "," : "");
            }
            std::cout << (python_literal ? "]" : "");
            std::cout << std::endl;
        }
        else{
            if constexpr(length(typename SPEC::SHAPE{}) == 2){
                if(python_literal){
                    std::cout << "[" << std::endl;
                }
                for(TI i=0; i < get<0>(typename SPEC::SHAPE{}); i++){
                    std::cout << (python_literal ? "[" : "");
                    for(TI j=0; j < get<1>(typename SPEC::SHAPE{}); j++){
                        T number = get(device, tensor, i, j);
                        std::cout <<  std::setw(15) << std::scientific << std::setprecision(6);
                        if constexpr (utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>){
                            std::cout << number;
                        }
                        else{
                            std::cout << (float)number;
                        }
                        if(python_literal){
                            std::cout << ", ";
                        }
                    }
                    std::cout << (python_literal ? "]," : "");
                    std::cout << std::endl;
                }
                std::cout << (python_literal ? "]" : "") << std::endl;
            }
            else{
                for(TI i=0; i < get<0>(typename SPEC::SHAPE{}); i++){
                    for(TI j=0; j < level; j++){
                        std::cout << " ";
                    }
                    std::cout << "dim[" << level << "] = " << i << ": " << std::endl;
                    auto v = view(device, tensor, i);
                    print(device, v, python_literal, level+1);
                }
            }
        }
    }
    template<typename DEV_SPEC, typename TI, TI VALUE, typename NEXT_ELEMENT >
    void print(devices::CPU<DEV_SPEC>& device, tensor::Element<TI, VALUE, NEXT_ELEMENT>, typename DEV_SPEC::index_t level=0, bool python_literal=false){
        using ELEMENT = tensor::Element<TI, VALUE, NEXT_ELEMENT>;
        if(level == 0){
            std::cout << "[";
        }
        if constexpr(utils::typing::is_same_v<typename NEXT_ELEMENT::NEXT_ELEMENT, tensor::FinalElement>){
            std::cout << VALUE << "]";
        }
        else{
            std::cout << VALUE << ", ";
            print(device, NEXT_ELEMENT{}, level+1, python_literal);
        }
        if (level==0){
            std::cout << std::endl;
        }
    }
    template<typename DEV_SPEC, typename SPEC>
    void print_serial(devices::CPU<DEV_SPEC>& device, const Tensor<SPEC>& tensor, bool print_index = false){
        using T = typename SPEC::T;
        T* data_pointer = data(tensor);
        for(typename DEV_SPEC::index_t i=0; i < SPEC::SIZE; i++){
            if(print_index){
                std::cout << i << ": ";
            }
            std::cout << data_pointer[i] << std::endl;
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
