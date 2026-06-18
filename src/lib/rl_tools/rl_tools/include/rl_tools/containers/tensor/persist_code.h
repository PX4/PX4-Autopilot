#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_TENSOR_PERSIST_CODE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_TENSOR_PERSIST_CODE_H

#include "../../persist/code.h"
#include "../../utils/generic/typing.h"
#include "../matrix/persist_code.h"
#include <sstream>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, Tensor<SPEC>& tensor, std::string name, bool const_declaration=true, typename DEVICE::index_t indent=0){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        static_assert(utils::typing::is_same_v<containers::persist::STORAGE_TYPE, unsigned char>);
        static_assert(sizeof(T) % sizeof(containers::persist::STORAGE_TYPE) == 0);
        std::stringstream indent_ss;
        for(TI i=0; i < indent; i++){
            indent_ss << "    ";
        }
        std::string ind = indent_ss.str();
        std::stringstream ss_header;
        ss_header << "// NOTE: This code export assumes that the endianness of the target platform is the same as the endianness of the source platform\n";
        ss_header << "#include <rl_tools/containers/tensor/tensor.h>\n";
        std::stringstream ss;
        ss << ind << "namespace " << name << " {\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << containers::persist::get_type_string<T>() << " memory[] = {";

        auto m = matrix_view(device, tensor);
        using MATRIX_SPEC = typename decltype(m)::SPEC;
        bool first = true;
        std::stringstream data_ss;
        data_ss << std::uppercase << std::hexfloat << std::setprecision(6);
        for(TI i=0; i < MATRIX_SPEC::ROWS; i++){
            for(TI j=0; j < MATRIX_SPEC::COLS; j++){
                if(!first){
                    data_ss << ", ";
                }
                auto value = get(m, i, j);
                first = false;
                if (!std::isfinite(value)) {
                    if (value < 0){
                        data_ss << '-';
                    }
                    data_ss << "0x0p+0f/* nan/inf */";
                } else {
                    data_ss << value << (utils::typing::is_same_v<T, float> ? "f" : "");
                }
            }
        }
        ss << data_ss.str();
        ss << "};\n";
        ss << ind << "    using SHAPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::Shape<" << containers::persist::get_type_string<TI>() << ", ";
        for(TI dim_i=0; dim_i < length(typename SPEC::SHAPE{}); dim_i++){
            if(dim_i > 0){
                ss << ", ";
            }
            ss << get(device, typename SPEC::SHAPE{}, dim_i);
        }
        ss << ">;\n";
        ss << ind << "    using SPEC = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::Specification<";
        ss << containers::persist::get_type_string<T>() << ", ";
        ss << containers::persist::get_type_string<TI>() << ", ";
        ss << "SHAPE, ";
        constexpr bool DYNAMIC_ALLOCATION = true; // it is not dynamically allocated but the statically allocated exported arrays are assigned to the pointer of the tensor. This is easier than initializing a static array inside the tensor directly
        ss << (DYNAMIC_ALLOCATION ? "true" : "false") << ", ";
        ss << "RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::tensor::RowMajorStride<SHAPE>, ";
        constexpr bool CONST = true;
        ss << (CONST ? "true" : "false");
        ss << ">;\n";
        ss << ind << "    using CONTAINER_TYPE = RL_TOOLS""_NAMESPACE_WRAPPER ::rl_tools::Tensor<SPEC>;\n";
        ss << ind << "    " << (const_declaration ? "constexpr " : "") << "CONTAINER_TYPE container = {(" << containers::persist::get_type_string<T>() << "*)" << "memory}; \n";
        ss << ind << "}\n";
        return {ss_header.str(), ss.str()};
    }
    template<typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device, Tensor<SPEC>& m, std::string name, bool const_declaration, typename DEVICE::index_t indent=0){
        auto code = save_code_split(device, m, name, const_declaration, indent);
        return code.header + code.body;
    }
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, Tensor<SPEC>& m){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        std::string data;
        data += "[";
        if constexpr(SPEC::SHAPE::LENGTH > 1){
            for(TI i=0; i < SPEC::SHAPE::template GET<0>; ++i){
                if (i != 0){
                    data += ", ";
                }
                auto next_m = view(device, m, i);
                data += json(device, next_m);
            }
        }
        else{
            for(TI i=0; i < SPEC::SHAPE::template GET<0>; i++){
                T value = get(device, m, i);
                if (i != 0){
                    data += ", ";
                }
                if (math::is_nan(device.math, value)) {
                    data += "null";
                }
                else{
                    data += std::to_string(value);
                }
            }
        }
        data += "]";
        return data;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
