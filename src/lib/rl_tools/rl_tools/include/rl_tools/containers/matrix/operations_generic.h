#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_GENERIC_H

#ifdef RL_TOOLS_CONTAINERS_USE_MALLOC
#include <stdlib.h>
#endif

#include "matrix.h"
#include "../../mode/mode.h"
#ifndef RL_TOOLS_FUNCTION_PLACEMENT
    #define RL_TOOLS_FUNCTION_PLACEMENT
#endif

#if defined(RL_TOOLS_DEBUG_CONTAINER_CHECK_BOUNDS) || defined(RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC)
#include <iostream>
#include <sstream>
#endif


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
//    template<typename DEVICE, typename SPEC>
//    void malloc(DEVICE& device, Matrix<SPEC>& matrix){
//#ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC
//        utils::assert_exit(device, matrix._data == nullptr, "Trying to malloc a MatrixStatic. Matrix is already allocated (stack)");
//#endif
////        matrix._data = (typename SPEC::T*)&matrix._data_memory[0];
//#ifdef RL_TOOLS_DEBUG_CONTAINER_MALLOC_INIT_NAN
//        for(typename SPEC::TI i = 0; i < SPEC::SIZE; i++){
//            if constexpr(utils::typing::is_same_v<typename SPEC::T, float> || utils::typing::is_same_v<typename SPEC::T, double>){
//                matrix._data[i] = math::nan<typename SPEC::T>(device.math);
//            }
//        }
//#endif
//    }
//    template<typename DEVICE, typename SPEC>
//    void free(DEVICE& device, Matrix<SPEC>& matrix){
//        // free is a no-op for statically allocated matrices
//    }
//
template<typename DEVICE, typename T, typename TI, TI SIZE>
RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, matrix::MatrixStatic<T, TI, SIZE>& matrix) {
    // no-op
}
template<typename DEVICE, typename T, typename TI, TI SIZE>
RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, matrix::MatrixStatic<T, TI, SIZE>& matrix) {
    // no-op
}
#if !defined(RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS)
    template<typename DEVICE, typename T, typename T_TI, T_TI SIZE_BYTES, bool T_CONST>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, matrix::MatrixDynamic<T, T_TI, SIZE_BYTES, T_CONST>& matrix){
        using TI = typename DEVICE::index_t;
#ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC
        utils::assert_exit(device, matrix._data == nullptr, "Matrix is already allocated");
#endif
#ifndef RL_TOOLS_DISABLE_ALIGNED_MEMORY_ALLOCATIONS
        static constexpr TI POINTER_SIZE = sizeof(void*);
        static constexpr TI BYTE_ALIGNMENT = 64;
        static constexpr TI ALIGNED_SIZE = SIZE_BYTES + BYTE_ALIGNMENT + POINTER_SIZE;
#else
        static constexpr TI ALIGNED_SIZE = SIZE_BYTES;
#endif
#ifdef RL_TOOLS_CONTAINERS_USE_MALLOC
        void* original_pointer = ::malloc(ALIGNED_SIZE);
#else
        char* original_pointer = new char[ALIGNED_SIZE];
#endif
#ifndef RL_TOOLS_DISABLE_ALIGNED_MEMORY_ALLOCATIONS
        char* byte_pointer = static_cast<char*>(original_pointer) + POINTER_SIZE;
        static_assert(sizeof(TI) >= sizeof(void*), "TI must be at least as large as a pointer");
        char* aligned_byte_pointer = reinterpret_cast<char*>((reinterpret_cast<TI>(byte_pointer) + BYTE_ALIGNMENT - 1) & ~(BYTE_ALIGNMENT - 1));
        char* original_pointer_storage = aligned_byte_pointer - POINTER_SIZE;
        *((decltype(original_pointer)*)original_pointer_storage) = original_pointer;
        matrix._data = reinterpret_cast<T*>(aligned_byte_pointer);
#else
        matrix._data = reinterpret_cast<T*>(original_pointer);
#endif


        count_malloc(device, SIZE_BYTES);

#ifdef RL_TOOLS_DEBUG_CONTAINER_MALLOC_INIT_NAN
        for(T_TI i = 0; i < static_cast<T_TI>(SIZE_BYTES/sizeof(T)); i++){
            if constexpr(utils::typing::is_same_v<T, float> || utils::typing::is_same_v<T, double>){
                matrix._data[i] = math::nan<T>(device.math);
            }
        }
#endif
    }
    template<typename DEVICE, typename T, typename T_TI, T_TI SIZE_BYTES, bool T_CONST>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, matrix::MatrixDynamic<T, T_TI, SIZE_BYTES, T_CONST>& matrix){
        using TI = typename DEVICE::index_t;
#ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC
        utils::assert_exit(device, matrix._data != nullptr, "Matrix has not been allocated");
#endif
#ifndef RL_TOOLS_DISABLE_ALIGNED_MEMORY_ALLOCATIONS
        char* aligned_byte_pointer = reinterpret_cast<char*>(matrix._data);
        static constexpr TI POINTER_SIZE = sizeof(void*);
        char* original_pointer_storage = aligned_byte_pointer - POINTER_SIZE;
    #ifdef RL_TOOLS_CONTAINERS_USE_MALLOC
            void* original_pointer = *((void**)original_pointer_storage);
            ::free(original_pointer);
    #else
            char* original_pointer = *((char**)original_pointer_storage);
            delete[] original_pointer;
    #endif
#else
        #ifdef RL_TOOLS_CONTAINERS_USE_MALLOC
            ::free(matrix._data);
        #else
            delete matrix._data;
        #endif
#endif
        matrix._data = nullptr;
    }
#endif

    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI rows(const Matrix<SPEC>& m){
        return SPEC::ROWS;
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI cols(const Matrix<SPEC>& m){
        return SPEC::COLS;
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI row_pitch(const Matrix<SPEC>& m){
        return SPEC::ROW_PITCH;
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr typename SPEC::TI col_pitch(const Matrix<SPEC>& m){
        return SPEC::COL_PITCH;
    }

    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr bool empty(const Matrix<SPEC>& m){
        return SPEC::COLS == 0 && SPEC::ROWS == 0;
    }

    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT inline typename SPEC::TI index(const Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        typename SPEC::TI index = row * row_pitch(m) + col * col_pitch(m);
        // bounds checking for debugging
#if defined(RL_TOOLS_DEBUG_CONTAINER_CHECK_BOUNDS)
        if(row >= SPEC::ROWS || col >= SPEC::COLS){
#if !defined(__CUDA_ARCH__)
            std::stringstream ss;
            ss << "index: " << row << "(" << SPEC::ROWS << "):" << col << "(" << SPEC::COLS << ") out of bounds";
            throw std::runtime_error(ss.str());
#else
            printf("index: %d(%d):%d(%d) out of bounds", (int)row, (int)SPEC::ROWS, (int)col, (int)SPEC::COLS);
#endif
        }
#endif
        return index;
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT inline utils::typing::conditional_t<SPEC::CONST, const typename SPEC::T&, typename SPEC::T&>  get(Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        return m._data[index(m, row, col)];
    }
    template<typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT inline const typename SPEC::T& get(const Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        return m._data[index(m, row, col)];
    }
    template<typename SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT inline void set(Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col, T value){
        m._data[index(m, row, col)] = static_cast<typename SPEC::T>(value);
    }
    template<typename SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT inline void increment(Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col, T value){
        m._data[index(m, row, col)] += value;
    }
    template<typename SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT inline void multiply(Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col, T value){
        m._data[index(m, row, col)] *= value;
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void transpose(DEVICE& device, Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        static_assert(SPEC_1::ROWS == SPEC_2::COLS);
        static_assert(SPEC_1::COLS == SPEC_2::ROWS);
        for(typename SPEC_1::TI i = 0; i < SPEC_1::ROWS; i++){
            for(typename SPEC_1::TI j = 0; j < SPEC_1::COLS; j++){
                set(target, i, j, get(source, j, i));
            }
        }
    }
    namespace containers::vectorization::operators{
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline T copy(DEVICE& dev, T b){
            return b;
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline T add(DEVICE& dev, T a, T b){
            return a+b;
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline T sub(DEVICE& dev, T a, T b){
            return a-b;
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline bool is_nan(DEVICE& dev, bool a, T c){
            return a || math::is_nan(dev, c);
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline bool is_finite(DEVICE& dev, bool a, T c){
            return a && math::is_finite(dev, c);
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline T max(DEVICE& dev, T a, T c){
            return math::max(dev, a, c);
        }
        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT inline T min(DEVICE& dev, T a, T c){
            return math::min(dev, a, c);
        }
    }
    template<auto ROWS, auto COLS, typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto reshape(DEVICE& device, Matrix<SPEC>& target){
        static_assert(SPEC::COL_PITCH == 1 || SPEC::ROW_PITCH == 1, "reshape: only contiguous matrices can be reshaped");
        static_assert((SPEC::COL_PITCH == 1 && SPEC::ROW_PITCH == SPEC::COLS) || (SPEC::ROW_PITCH == 1 && SPEC::COL_PITCH == SPEC::ROWS), "reshape: only contiguous matrices can be reshaped");
        static_assert(SPEC::ROWS * SPEC::COLS == ROWS * COLS, "reshape: new size must match old size");
        using TI = typename SPEC::TI;
        using Layout = matrix::layouts::Fixed<TI, SPEC::COL_PITCH == 1 ? COLS : 1, SPEC::COL_PITCH == 1 ? 1 : ROWS>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, ROWS, COLS, true, Layout>> out;
        out._data = target._data;
        return out;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto view_transpose(DEVICE& device, Matrix<SPEC>& target){
//        static_assert(SPEC::ROWS == SPEC::COLS);
        using TI = typename SPEC::TI;
//        for(TI i = 0; i < SPEC::ROWS; i++){
//            for(TI j = i + 1; j < SPEC::COLS; j++){
//                T temp = get(target, i,  j);
//                set(target,  i,  j, get(target,  j,  i));
//                set(target,  j,  i, temp);
//            }
//        }
        using LayOut = matrix::layouts::Fixed<TI, SPEC::COL_PITCH, SPEC::ROW_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::COLS, SPEC::ROWS, true, LayOut>> out;
        out._data = target._data;
        return out;
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC_1::T abs_diff(DEVICE& device, const Matrix<SPEC_1>& m1, const Matrix<SPEC_2>& m2){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using SPEC = SPEC_1;
        using T = typename SPEC::T;
        typename SPEC::T acc = 0;
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                T v1 = get(m1, i, j);
                T v2 = get(m2, i, j);
                acc += math::abs(device.math, v1 - v2);
            }
        }
        return acc;
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_3, auto BINARY_OPERATOR>
    RL_TOOLS_FUNCTION_PLACEMENT void vectorize_binary(DEVICE& device, const Matrix<SPEC_1>& a, const Matrix<SPEC_2>& b, Matrix<SPEC_3>& c){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        static_assert(containers::check_structure<SPEC_2, SPEC_3>);
        using SPEC = SPEC_1;
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                set(c, i, j, BINARY_OPERATOR(get(a, i, j), get(b, i, j)));
            }
        }
    }
    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC, auto UNARY_OPERATOR>
    RL_TOOLS_FUNCTION_PLACEMENT void vectorize_unary(DEVICE& device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
        using SPEC = SOURCE_SPEC;
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                set(target, i, j, UNARY_OPERATOR(device.math, get(source, i, j)));
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename RETURN_TYPE, auto BINARY_OPERATOR>
    RL_TOOLS_FUNCTION_PLACEMENT RETURN_TYPE reduce_unary(DEVICE& device, const Matrix<SPEC>& source, const RETURN_TYPE& init){
        using TI = typename SPEC::TI;
        RETURN_TYPE acc = init;
        for(TI row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(TI col_i = 0; col_i < SPEC::COLS; col_i++){
                acc = BINARY_OPERATOR(device.math, acc, get(source, row_i, col_i));
            }
        }
        return acc;
    }
    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename RETURN_TYPE, auto TERTIARY_OPERATOR>
    RL_TOOLS_FUNCTION_PLACEMENT RETURN_TYPE reduce_binary(DEVICE& device, Matrix<SPEC_1>& source_1, Matrix<SPEC_2>& source_2, const RETURN_TYPE& init){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using TI = typename SPEC_1::TI;
        RETURN_TYPE acc = init;
        for(TI row_i = 0; row_i < SPEC_1::ROWS; row_i++){
            for(TI col_i = 0; col_i < SPEC_1::COLS; col_i++){
                acc = TERTIARY_OPERATOR(device.math, acc, get(source_1, row_i, col_i), get(source_2, row_i, col_i));
            }
        }
        return acc;
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void add(DEVICE& device, const  Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        using SPEC = SPEC_1;
        vectorize_binary<DEVICE, SPEC_1, SPEC_2, SPEC_1, containers::vectorization::operators::add<typename SPEC::T>>(device, target, source, target);
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_3>
    RL_TOOLS_FUNCTION_PLACEMENT void sub(DEVICE& device, const Matrix<SPEC_1>& a, const Matrix<SPEC_2>& b, Matrix<SPEC_3>& c){
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        static_assert(containers::check_structure<SPEC_2, SPEC_3>);
        using SPEC = SPEC_1;
        vectorize_binary<DEVICE, SPEC_1, SPEC_2, SPEC_3, containers::vectorization::operators::sub<typename SPEC::T>>(device, a, b, c);
    }

//    template<typename DEVICE, typename SPEC_1, typename SPEC_2>
//    RL_TOOLS_FUNCTION_PLACEMENT void add_broadcast(DEVICE& device, const Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
//        static_assert(SPEC_1::COLS == SPEC_2::COLS);
//        static_assert(SPEC_2::ROWS == 1);
//        using SPEC = SPEC_1;
//        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
//            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
//                get(target, i, j) += get(source, 0, j);
//            }
//        }
//    }
    template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void set_broadcast(DEVICE& device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        static_assert(SOURCE_SPEC::ROWS == 1);
        static_assert(TARGET_SPEC::COLS == SOURCE_SPEC::COLS);
        using SPEC = TARGET_SPEC;
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                set(target, i, j, get(source, 0, j));
            }
        }
    }

    template<typename DEVICE, typename SPEC, typename VALUE_T>
    RL_TOOLS_FUNCTION_PLACEMENT void set_all(DEVICE& device, Matrix<SPEC>& m, VALUE_T value){
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                set(m, i, j, value);
            }
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_all(DEVICE& device, Matrix<SPEC>& m, typename SPEC::T factor){
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                multiply(m, i, j, factor);
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename VALUE_T>
    RL_TOOLS_FUNCTION_PLACEMENT void increment_all(DEVICE& device, Matrix<SPEC>& m, VALUE_T value){
        for(typename SPEC::TI i = 0; i < SPEC::ROWS; i++){
            for(typename SPEC::TI j = 0; j < SPEC::COLS; j++){
                increment(m, i, j, value);
            }
        }
    }

    template<typename DEVICE, typename SPEC_1, typename SPEC_2, typename SPEC_3>
    RL_TOOLS_FUNCTION_PLACEMENT void mul(DEVICE& device, const Matrix<SPEC_1>& A, const Matrix<SPEC_2>& B, const Matrix<SPEC_3>& C){
        static_assert(SPEC_1::COLS == SPEC_2::ROWS);
        static_assert(SPEC_1::ROWS == SPEC_3::ROWS);
        static_assert(SPEC_2::COLS == SPEC_3::COLS);
        using SPEC = SPEC_1;
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        for(TI i = 0; i < SPEC::ROWS; i++){
            for(TI j = 0; j < SPEC::COLS; j++){
                T acc = 0;
                for(TI k = 0; k < SPEC::COLS; k++){
                    acc += get(A, i, k) * get(B, k, j);
                }
                get(C, i, j) = acc;
            }
        }
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_C>
    RL_TOOLS_FUNCTION_PLACEMENT void hcat(DEVICE& device, const Matrix<SPEC_A>& A, const Matrix<SPEC_B>& B, Matrix<SPEC_C>& C){
        static_assert(SPEC_A::ROWS == SPEC_B::ROWS);
        static_assert(SPEC_C::ROWS == SPEC_A::ROWS);
        static_assert(SPEC_A::COLS + SPEC_B::COLS == SPEC_C::COLS);
        // concatenate horizontally
        using TI = typename SPEC_A::TI;
        for(TI i = 0; i < SPEC_A::ROWS; i++){
            for(TI j = 0; j < SPEC_A::COLS; j++){
                set(C, i, j, get(A, i, j));
            }
            for(TI j = 0; j < SPEC_B::COLS; j++){
                set(C, i, (j + SPEC_A::COLS), get(B, i, j));
            }
        }
    }
    // vcat
    template<typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_C>
    RL_TOOLS_FUNCTION_PLACEMENT void vcat(DEVICE& device, const Matrix<SPEC_A>& A, const Matrix<SPEC_B>& B, const Matrix<SPEC_C>& C){
        static_assert(SPEC_A::COLS == SPEC_B::COLS);
        static_assert(SPEC_C::COLS == SPEC_A::COLS);
        static_assert(SPEC_A::ROWS + SPEC_B::ROWS == SPEC_C::ROWS);
        // concatenate horizontally
        using TI = typename SPEC_A::TI;
        for(TI i = 0; i < SPEC_A::ROWS; i++){
            for(TI j = 0; j < SPEC_A::COLS; j++){
                set(C, i, j, get(A, i, j));
            }
        }
        for(TI i = 0; i < SPEC_B::ROWS; i++){
            for(TI j = 0; j < SPEC_B::COLS; j++){
                set(C, i + SPEC_A::ROWS, j, get(B, i, j));
            }
        }
    }
//    template<typename DEVICE, typename SPEC_1, typename SPEC_2, bool BOUNDS_CHECKING=true>
//    RL_TOOLS_FUNCTION_PLACEMENT void slice(DEVICE& device, const  Matrix<SPEC_1>& source, Matrix<SPEC_2>& target, typename SPEC_1::TI row, typename SPEC_1::TI col, typename SPEC_1::TI rows = SPEC_1::ROWS, typename SPEC_1::TI cols = SPEC_1::COLS, typename SPEC_1::TI target_row=0, typename SPEC_1::TI target_col=0){
////        static_assert(SPEC_1::ROWS <= SPEC_2::ROWS);
////        static_assert(SPEC_1::COLS <= SPEC_2::COLS);
//        using TI = typename SPEC_1::TI;
//        if constexpr(BOUNDS_CHECKING){
//            utils::assert_exit(device, row + rows <= SPEC_2::ROWS, "row + rows <= SPEC_2::ROWS");
//            utils::assert_exit(device, col + cols <= SPEC_2::COLS, "col + cols <= SPEC_2::COLS");
//            utils::assert_exit(device, target_row + rows <= SPEC_1::ROWS, "target_row + rows <= SPEC_1::ROWS");
//            utils::assert_exit(device, target_col + cols <= SPEC_1::COLS, "target_col + cols <= SPEC_1::COLS");
//        }
//        for(TI i = 0; i < rows; i++){
//            for(TI j = 0; j < cols; j++){
//                set(target, target_row + i, target_col + j, get(source, row + i, col + j));
//            }
//        }
//    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T sum(DEVICE& device, const Matrix<SPEC>& m){
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T acc = 0;
        for(TI i = 0; i < SPEC::ROWS; i++){
            for(TI j = 0; j < SPEC::COLS; j++){
                acc += get(m, i, j);
            }
        }
        return acc;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T sum_of_squares(DEVICE& device, const Matrix<SPEC>& m){
        using TI = typename SPEC::TI;
        using T = typename SPEC::T;
        T acc = 0;
        for(TI i = 0; i < SPEC::ROWS; i++){
            for(TI j = 0; j < SPEC::COLS; j++){
                auto val = get(m, i, j);
                acc += val * val;
            }
        }
        return acc;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T mean_of_squares(DEVICE& device, const Matrix<SPEC>& m){
        return sum_of_squares(device, m) / (SPEC::ROWS * SPEC::COLS);
    }
    template<typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
//    [[deprecated("Note: math::isnan might be optimized out by the compiler when using e.g. fast-math")]]
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const Matrix<SPEC>& m, const Mode<MODE>& mode = {}){
        return reduce_unary<DEVICE, SPEC, bool, containers::vectorization::operators::is_nan<typename DEVICE::SPEC::MATH, typename SPEC::T>>(device, m, false);
    }
    template<typename DEVICE, typename SPEC>
//    [[deprecated("Note: math::isfinite might be optimized out by the compiler when using e.g. fast-math")]]
    bool is_finite(DEVICE& device, const Matrix<SPEC>& m){
        return reduce_unary<DEVICE, SPEC, bool, containers::vectorization::operators::is_finite<typename DEVICE::SPEC::MATH, typename SPEC::T>>(device, m, true);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T max(DEVICE& device, const Matrix<SPEC>& m){
        static_assert(SPEC::ROWS > 0 && SPEC::COLS > 0);
        using T = typename SPEC::T;
        T init = get(m, 0, 0);
        return reduce_unary<DEVICE, SPEC, T, containers::vectorization::operators::max<typename DEVICE::SPEC::MATH, typename SPEC::T>>(device, m, init);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T min(DEVICE& device, const Matrix<SPEC>& m){
        static_assert(SPEC::ROWS > 0 && SPEC::COLS > 0);
        using T = typename SPEC::T;
        T init = get(m, 0, 0);
        return reduce_unary<DEVICE, SPEC, T, containers::vectorization::operators::min<typename DEVICE::SPEC::MATH, typename SPEC::T>>(device, m, init);
    }
    template<typename SOURCE_DEVICE, typename SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(SOURCE_DEVICE& source_device, const T* source, Matrix<SPEC>& target, typename SPEC::TI row = 0, typename SPEC::TI col = 0, typename SPEC::TI rows = SPEC::ROWS, typename SPEC::TI cols = SPEC::COLS, typename SPEC::TI row_pitch = SPEC::COLS, typename SPEC::TI col_pitch = 1){
        using TI = typename SPEC::TI;
        utils::assert_exit(source_device, row + rows <= SPEC::ROWS, "row + rows <= SPEC::ROWS");
        utils::assert_exit(source_device, col + cols <= SPEC::COLS, "col + cols <= SPEC::COLS");
        for(TI i = 0; i < rows; i++){
            for(TI j = 0; j < cols; j++){
                set(target, row + i, col+j, source[i * row_pitch + j * col_pitch]);
            }
        }
    }

    template<typename SOURCE_DEVICE, typename SPEC, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(SOURCE_DEVICE& target_device, Matrix<SPEC>& source, T* target, typename SPEC::TI row = 0, typename SPEC::TI col = 0, typename SPEC::TI rows = SPEC::ROWS, typename SPEC::TI cols = SPEC::COLS, typename SPEC::TI row_pitch = SPEC::COLS, typename SPEC::TI col_pitch = 1){
        using TI = typename SPEC::TI;
        utils::assert_exit(target_device, row + rows <= SPEC::ROWS, "row + rows <= SPEC::ROWS");
        utils::assert_exit(target_device, col + cols <= SPEC::COLS, "col + cols <= SPEC::COLS");
        for(TI i = 0; i < rows; i++){
            for(TI j = 0; j < cols; j++){
//                set(source, row + i, col+j, target[i * row_pitch + j * col_pitch]);
                target[i * cols + j] = get(source, row + i, col+j);
            }
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, Matrix<SPEC>& m){
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::ROWS, SPEC::COLS, true, ViewLayout, false>> out;
        out._data = m._data;
        return out;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, const Matrix<SPEC>& m){
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        const Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::ROWS, SPEC::COLS, true, ViewLayout, true>> out;
        out._data = m._data;
        return out;
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, Matrix<SPEC>& m){
        static_assert(SPEC::ROWS >= ROWS);
        static_assert(SPEC::COLS >= COLS);
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, ROWS, COLS, true, ViewLayout, false>> out;
        out._data = m._data;
        return out;
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, const Matrix<SPEC>& m){
        static_assert(SPEC::ROWS >= ROWS);
        static_assert(SPEC::COLS >= COLS);
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        const Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, ROWS, COLS, true, ViewLayout, true>> out;
        out._data = m._data;
        return out;
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT auto _view(DEVICE& device, Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        static_assert(SPEC::ROWS >= ROWS);
        static_assert(SPEC::COLS >= COLS);
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, ROWS, COLS, true, ViewLayout, false>> out;
        out._data = m._data + row * row_pitch(m) + col * col_pitch(m);
        return out;
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT auto _view(DEVICE& device, const Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        static_assert(SPEC::ROWS >= ROWS);
        static_assert(SPEC::COLS >= COLS);
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        const Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, ROWS, COLS, true, ViewLayout, true>> out;
        out._data = m._data + row * row_pitch(m) + col * col_pitch(m);
        return out;
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT auto view(DEVICE& device, Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        return _view<DEVICE, SPEC, ROWS, COLS>(device, m, row, col); // so that the CPU implementation can reuse _view while doing runtime bounds checking if wished for
    }
    template<typename DEVICE, typename SPEC, typename SPEC::TI ROWS, typename SPEC::TI COLS>
    RL_TOOLS_FUNCTION_PLACEMENT auto view(DEVICE& device, const Matrix<SPEC>& m, typename SPEC::TI row, typename SPEC::TI col){
        return _view<DEVICE, SPEC, ROWS, COLS>(device, m, row, col); // so that the CPU implementation can reuse _view while doing runtime bounds checking if wished for
    }
    template<typename DEVICE, typename SPEC, typename ViewSpec>
    RL_TOOLS_FUNCTION_PLACEMENT auto view(DEVICE& device, Matrix<SPEC>& m, const ViewSpec& vs, typename SPEC::TI row, typename SPEC::TI col){
        return view<DEVICE, SPEC, ViewSpec::ROWS, ViewSpec::COLS>(device, m, row, col);
    }
    template<typename DEVICE, typename SPEC, typename ViewSpec>
    RL_TOOLS_FUNCTION_PLACEMENT auto view(DEVICE& device, const Matrix<SPEC>& m, const ViewSpec& vs, typename SPEC::TI row, typename SPEC::TI col){
        return view<DEVICE, SPEC, ViewSpec::ROWS, ViewSpec::COLS>(device, m, row, col);
    }
    template<typename DEVICE, typename SPEC, typename ViewSpec>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, Matrix<SPEC>& m, const ViewSpec& vs){
        return view<DEVICE, SPEC, ViewSpec::ROWS, ViewSpec::COLS>(device, m);
    }
    template<typename DEVICE, typename SPEC, typename ViewSpec>
    RL_TOOLS_FUNCTION_PLACEMENT constexpr auto view(DEVICE& device, const Matrix<SPEC>& m, const ViewSpec& vs){
        return view<DEVICE, SPEC, ViewSpec::ROWS, ViewSpec::COLS>(device, m);
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto row(DEVICE& device, Matrix<SPEC>& m, typename SPEC::TI row){
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, 1, SPEC::COLS, true, ViewLayout, false>> out;
        out._data = m._data + row * row_pitch(m);
        return out;
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto row(DEVICE& device, const Matrix<SPEC>& m, typename SPEC::TI row){
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        const Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, 1, SPEC::COLS, true, ViewLayout, true>> out{m._data + row * row_pitch(m)};
        return out;
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto col(DEVICE& device, const Matrix<SPEC>& m, typename SPEC::TI col){
        using ViewLayout = matrix::layouts::Fixed<typename SPEC::TI, SPEC::ROW_PITCH, SPEC::COL_PITCH>;
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::ROWS, 1, true, ViewLayout, true>> out;
        out._data = m._data + col * col_pitch(m);
        return out;
    }

    template <typename DEVICE, typename T, typename INPUT_SPEC, typename MEAN_SPEC, typename STD_SPEC, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void standardise(DEVICE& device, const rl_tools::Matrix<INPUT_SPEC>& input, const rl_tools::Matrix<MEAN_SPEC> mean, const rl_tools::Matrix<STD_SPEC> std, rl_tools::Matrix<OUTPUT_SPEC> output){
        static_assert(rl_tools::containers::check_structure<INPUT_SPEC, OUTPUT_SPEC>);
        static_assert(rl_tools::containers::check_structure<MEAN_SPEC, STD_SPEC>);
        static_assert(INPUT_SPEC::COLS == MEAN_SPEC::COLS);
        static_assert(MEAN_SPEC::ROWS == 1);
        for(typename DEVICE::index_t row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < INPUT_SPEC::COLS; col_i++){
                set(output, row_i, col_i, (get(input, row_i, col_i) - get(mean, 0, col_i)) / get(std, 0, col_i));
            }
        }
    }

    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void randn(DEVICE& device, rl_tools::Matrix<SPEC>& m, RNG& rng){
        using T = typename SPEC::T;
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                set(m, row_i, col_i, random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM(), (T)0, (T)1, rng));
            }
        }
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T mean(DEVICE& device, rl_tools::Matrix<SPEC>& m){
        using T = typename SPEC::T;
        T acc = 0;
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                acc += get(m, row_i, col_i);
            }
        }
        return acc/(SPEC::ROWS * SPEC::COLS);
    }
    template <typename DEVICE, typename SPEC, typename OUTPUT_SPEC_1, typename OUTPUT_SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void mean_std_colwise(DEVICE& device, rl_tools::Matrix<SPEC>& m, rl_tools::Matrix<OUTPUT_SPEC_1>& mean, rl_tools::Matrix<OUTPUT_SPEC_2>& std){
        static_assert(SPEC::COLS == OUTPUT_SPEC_1::COLS);
        static_assert(SPEC::COLS == OUTPUT_SPEC_2::COLS);
        static_assert(SPEC::ROWS >= 2);
        using T = typename SPEC::T;
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                if(row_i == 0){
                    set(mean, 0, col_i, 0);
                    set(std, 0, col_i, 0);
                }
                T current_value = get(m, row_i, col_i);
                increment(mean, 0, col_i, current_value);
                increment(std, 0, col_i, current_value * current_value);
            }
        }
        for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
            set(mean, 0, col_i, get(mean, 0, col_i) / SPEC::ROWS);
            set(std, 0, col_i, math::sqrt(device.math, get(std, 0, col_i) / SPEC::ROWS - get(mean, 0, col_i) * get(mean, 0, col_i)));
        }
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T std(DEVICE& device, rl_tools::Matrix<SPEC>& m){
        static_assert(SPEC::ROWS * SPEC::COLS > 1);
        using T = typename SPEC::T;
        T acc = 0;
        T avg = mean(device, m);
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                T diff = get(m, row_i, col_i) - avg;
                acc += diff * diff;
            }
        }
        if(acc < 1e-6){
            return 0;
        }
        else{
            return math::sqrt(device.math, acc/(SPEC::ROWS * SPEC::COLS - 1));
        }
    }

    template <typename DEVICE, typename T, typename DEVICE::index_t DIM>
    Matrix<matrix::Specification<T, typename DEVICE::index_t, 1, DIM, true, matrix::layouts::RowMajorAlignment<typename DEVICE::index_t, 1>>> wrap(DEVICE& dev, T* data){
        return {data};
    }

    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void clamp(DEVICE& device, rl_tools::Matrix<SPEC>& m, typename SPEC::T lower, typename SPEC::T upper){
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                set(m, row_i, col_i, math::clamp<typename SPEC::T>(device.math, get(m, row_i, col_i), lower, upper));
            }
        }
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT void swap(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b){
        using T = typename SPEC_A::T;
        using TI = typename DEVICE::index_t;
        static_assert(containers::check_structure<SPEC_A, SPEC_B>);
        for(TI row_i = 0; row_i < SPEC_A::ROWS; row_i++){
            for(TI col_i = 0; col_i < SPEC_A::COLS; col_i++){
                T tmp = get(a, row_i, col_i);
                set(a, row_i, col_i, get(b, row_i, col_i));
                set(b, row_i, col_i, tmp);
            }
        }
    }
    template<typename DEVICE, typename SPEC_A, typename SPEC_B>
    RL_TOOLS_FUNCTION_PLACEMENT void swap(DEVICE& device, Matrix<SPEC_A>& a, Matrix<SPEC_B>& b, typename DEVICE::index_t row_a, typename DEVICE::index_t col_a, typename DEVICE::index_t row_b, typename DEVICE::index_t col_b){
        using T = typename SPEC_A::T;
        static_assert(containers::check_structure<SPEC_A, SPEC_B>);
        T tmp = get(a, row_a, col_a);
        set(a, row_a, col_a, get(b, row_b, col_b));
        set(b, row_b, col_b, tmp);
    }
    template <typename DEVICE, typename INPUT_SPEC, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void normalize(DEVICE& device, Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output){
        static_assert(containers::check_structure<INPUT_SPEC, OUTPUT_SPEC>);
        using T = typename INPUT_SPEC::T;
        using TI = typename DEVICE::index_t;
        T mu = mean(device, input);
        T std = math::sqrt(device.math, mean_of_squares(device, input) - mu * mu);
        for(TI row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            for(TI col_i = 0; col_i < INPUT_SPEC::COLS; col_i++){
                T x = get(input, row_i, col_i);
                T normalized = (x - mu) / std;
                set(output, row_i, col_i, normalized);
            }
        }
    }
    template <typename DEVICE, typename INPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void normalize(DEVICE& device, Matrix<INPUT_SPEC>& input){
        normalize(device, input, input);
    }

    template <typename DEVICE, typename MEAN_SPEC, typename STD_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void normalize(DEVICE& device, Matrix<MEAN_SPEC>& mean, Matrix<STD_SPEC>& std, Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output){
        static_assert(containers::check_structure<MEAN_SPEC, STD_SPEC>);
        static_assert(containers::check_structure<INPUT_SPEC, OUTPUT_SPEC>);
        static_assert(MEAN_SPEC::ROWS == 1);
        static_assert(MEAN_SPEC::COLS == INPUT_SPEC::COLS);

        using T = typename INPUT_SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI DATA_SIZE = INPUT_SPEC::ROWS;
        constexpr TI DIM = INPUT_SPEC::COLS;
        for(TI row_i = 0; row_i < DATA_SIZE; row_i++){
            for(TI col_i = 0; col_i < DIM; col_i++){
                T x = get(input, row_i, col_i);
                T mu = get(mean, 0, col_i);
                T sigma = get(std, 0, col_i);
                T normalized_x = (x - mu) / sigma;
                set(output, row_i, col_i, normalized_x);
            }
        }
    }
    template <typename DEVICE, typename MEAN_SPEC, typename STD_SPEC, typename INPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void normalize(DEVICE& device, Matrix<MEAN_SPEC>& mean, Matrix<STD_SPEC>& std, Matrix<INPUT_SPEC>& m){
        normalize(device, mean, std, m, m);
    }
    template <typename DEVICE, typename SPEC_INPUT, typename SPEC_OUTPUT>
    RL_TOOLS_FUNCTION_PLACEMENT void argmax_row_wise(DEVICE& device, Matrix<SPEC_INPUT>& input, Matrix<SPEC_OUTPUT>& output){
        static_assert(SPEC_INPUT::ROWS == SPEC_OUTPUT::ROWS);
        static_assert(SPEC_OUTPUT::COLS == 1);
        using T = typename SPEC_INPUT::T;
        using TI = typename DEVICE::index_t;

        for(TI row_i = 0; row_i < SPEC_INPUT::ROWS; row_i++){
            T max = 0;
            TI argmax = 0;
            for(TI col_i = 0; col_i < SPEC_INPUT::COLS; col_i++){
                if(col_i == 0){
                    max = get(input, row_i, col_i);
                    argmax = col_i;
                }
                else{
                    T value = get(input, row_i, col_i);
                    if(value > max){
                        max = value;
                        argmax = col_i;
                    }
                }
            }
            set(output, row_i, 0, argmax);
        }
    }
    template <typename DEVICE, typename SPEC_INPUT>
    RL_TOOLS_FUNCTION_PLACEMENT typename DEVICE::index_t argmax_row(DEVICE& device, Matrix<SPEC_INPUT>& input){
        static_assert(SPEC_INPUT::ROWS == 1);
        using T = typename SPEC_INPUT::T;
        using TI = typename DEVICE::index_t;
        Matrix<matrix::Specification<TI, TI, 1, 1, false>> output;
        argmax_row_wise(device, input, output);
        auto result = get(output, 0, 0);
        return result;
    }

    template <typename DEVICE, typename SPEC_INPUT, typename SPEC_OUTPUT>
    RL_TOOLS_FUNCTION_PLACEMENT void argmax_col_wise(DEVICE& device, Matrix<SPEC_INPUT>& input, Matrix<SPEC_OUTPUT>& output){
        static_assert(SPEC_INPUT::ROWS == SPEC_OUTPUT::ROWS);
        static_assert(SPEC_OUTPUT::COLS == 1);
        using T = typename SPEC_INPUT::T;
        using TI = typename DEVICE::index_t;

        for(TI col_i = 0; col_i < SPEC_INPUT::COLS; col_i++){
            T max = 0;
            TI argmax = 0;
            for(TI row_i = 0; row_i < SPEC_INPUT::ROWS; row_i++){
                if(col_i == 0){
                    max = get(input, row_i, col_i);
                    argmax = row_i;
                }
                else{
                    T value = get(input, row_i, col_i);
                    if(value > max){
                        max = value;
                        argmax = row_i;
                    }
                }
            }
            set(output, col_i, 0, argmax);
        }
    }

    template <typename DEVICE, typename SPEC_INPUT>
    RL_TOOLS_FUNCTION_PLACEMENT typename DEVICE::index_t argmax_col(DEVICE& device, Matrix<SPEC_INPUT>& input){
        static_assert(SPEC_INPUT::COLS == 1);
        using T = typename SPEC_INPUT::T;
        using TI = typename DEVICE::index_t;
        Matrix<matrix::Specification<TI, TI, 1, 1, false>> output;
        argmax_col_wise(device, input, output);
        auto result = get(output, 0, 0);
        free(device, output);
        return result;
    }
    namespace containers::matrix{
        template <typename T, unsigned M, unsigned N, unsigned K, unsigned LDA, unsigned LDB, unsigned LDC, bool ACCUMULATE>
        void generic_gemm_kernel(const T* __restrict A, const T* __restrict B, T* __restrict C){
            if constexpr(!ACCUMULATE){
                for(unsigned i = 0; i < M; ++i){
                    for(unsigned j = 0; j < N; ++j){
                        C[i * LDC + j] = 0;
                    }
                }
            }
            for(unsigned i = 0; i < M; ++i){
                for(unsigned j = 0; j < N; ++j){
                    for(unsigned k = 0; k < K; ++k){
                        C[i * LDC + j] += A[i * LDA + k] * B[k * LDB + j];
                    }
                }
            }
        }

    }
    template<bool ACCUMULATE, typename DEVICE, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_generic(DEVICE& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output) {
        static_assert(INPUT_SPEC_A::ROWS == OUTPUT_SPEC::ROWS);
        static_assert(INPUT_SPEC_A::COLS == INPUT_SPEC_B::ROWS);
        static_assert(INPUT_SPEC_B::COLS == OUTPUT_SPEC::COLS);

        using T = typename OUTPUT_SPEC::T;
        using TI = typename DEVICE::index_t;

        constexpr bool UNIFORM_TYPE = utils::typing::is_same_v<typename INPUT_SPEC_A::T, typename INPUT_SPEC_B::T> && utils::typing::is_same_v<typename INPUT_SPEC_A::T, typename OUTPUT_SPEC::T>;
        constexpr bool ROW_MAJOR = INPUT_SPEC_A::ROW_MAJOR && INPUT_SPEC_B::ROW_MAJOR && OUTPUT_SPEC::ROW_MAJOR;
        if constexpr(UNIFORM_TYPE && ROW_MAJOR){
            constexpr TI M = INPUT_SPEC_A::ROWS;
            constexpr TI N = INPUT_SPEC_B::COLS;
            constexpr TI K = INPUT_SPEC_A::COLS;
            constexpr TI LDA = INPUT_SPEC_A::ROW_PITCH;
            constexpr TI LDB = INPUT_SPEC_B::ROW_PITCH;
            constexpr TI LDC = OUTPUT_SPEC::ROW_PITCH;
            containers::matrix::generic_gemm_kernel<typename INPUT_SPEC_A::T, M, N, K, LDA, LDB, LDC, ACCUMULATE>(A._data, B._data, output._data);
        }
        else {
            for(TI row_i = 0; row_i < OUTPUT_SPEC::ROWS; row_i++){
                for(TI col_i = 0; col_i < OUTPUT_SPEC::COLS; col_i++){
                    T acc = 0;
                    if constexpr(ACCUMULATE){
                        acc = get(output, row_i, col_i);
                    }
                    for(TI k = 0; k < INPUT_SPEC_A::COLS; k++){
                        acc += get(A, row_i, k) * get(B, k, col_i);
                    }
                    set(output, row_i, col_i, acc);
                }
            }
        }

    }
    template<typename DEVICE, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply(DEVICE& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        multiply_generic<false>(device, A, B, output);
    }
    template<typename DEVICE, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void multiply_accumulate(DEVICE& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        multiply_generic<true>(device, A, B, output);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto matrix_view(DEVICE& device, Matrix<SPEC>& m){
        Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::ROWS, SPEC::COLS, true>> out{m._data};
        return out;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto matrix_view(DEVICE& device, const Matrix<SPEC>& m){
        const Matrix<matrix::Specification<typename SPEC::T, typename SPEC::TI, SPEC::ROWS, SPEC::COLS, true>> out{m._data};
        return out;
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T squared_sum(DEVICE& device, const Matrix<SPEC>& m){
        typename SPEC::T acc = 0;
        for(typename DEVICE::index_t row_i = 0; row_i < SPEC::ROWS; row_i++){
            for(typename DEVICE::index_t col_i = 0; col_i < SPEC::COLS; col_i++){
                auto val = get(m, row_i, col_i);
                acc += val * val;
            }
        }
        return acc;
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SPEC_1, typename SPEC_2>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const Matrix<SPEC_1>& source, Matrix<SPEC_2>& target){
        using TI = typename SOURCE_DEVICE::index_t;
        using T = typename SPEC_1::T;
        static_assert(containers::check_structure<SPEC_1, SPEC_2>);
        for (TI row_i=0; row_i < SPEC_1::ROWS; row_i++){
            for (TI col_i=0; col_i < SPEC_1::COLS; col_i++){
                T value = get(source, row_i, col_i);
                set(target, row_i, col_i, value);
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
