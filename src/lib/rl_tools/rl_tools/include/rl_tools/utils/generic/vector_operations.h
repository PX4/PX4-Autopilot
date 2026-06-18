#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_UTILS_GENERIC_VECTOR_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_UTILS_GENERIC_VECTOR_OPERATIONS_GENERIC_H

#ifndef RL_TOOLS_FUNCTION_PLACEMENT
#define RL_TOOLS_FUNCTION_PLACEMENT
#endif

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::utils::vector_operations{
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void scalar_multiply(const T v[N], const T s, T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] = v[i]*s;
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void scalar_multiply(T v[N], const T s) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            v[i] *= s;
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void scalar_multiply_accumulate(const T v[N], T s, T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] += v[i]*s;
        }
    }

    template <typename DEVICE, typename T, auto M, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void matrix_vector_product(const T A[M][N], const T v[N], T out[M]) {
        for(typename DEVICE::index_t i = 0; i < M; i++) {
            out[i] = 0;
            for(typename DEVICE::index_t j = 0; j < N; j++) {
                out[i] += A[i][j]*v[j];
            }
        }
    }

    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void cross_product(const T v1[3], const T v2[3], T out[3]) {
        // flops: 2 * 3 = 6
        out[0] = v1[1]*v2[2] - v1[2]*v2[1];
        out[1] = v1[2]*v2[0] - v1[0]*v2[2];
        out[2] = v1[0]*v2[1] - v1[1]*v2[0];
    }

    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void cross_product_accumulate(const T v1[3], const T v2[3], T out[3]) {
        out[0] += v1[1]*v2[2] - v1[2]*v2[1];
        out[1] += v1[2]*v2[0] - v1[0]*v2[2];
        out[2] += v1[0]*v2[1] - v1[1]*v2[0];
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void add(const T v1[N], const T v2[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] = v1[i] + v2[i];
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void add_accumulate(const T v1[N], const T v2[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] += v1[i] + v2[i];
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void add_accumulate(T const v[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] += v[i];
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void sub(T const v1[N], const T v2[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] = v1[i] - v2[i];
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void sub(T const v1[N], const T v2, T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] = v1[i] - v2;
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void sub_accumulate(const T v1[N], const T v2[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] += v1[i] - v2[i];
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void sub_accumulate(const T v[N], T out[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            out[i] -= v[i];
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void fill(T v[N], T s) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            v[i] = s;
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(const T source[N], T target[N]) {
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            target[i] = source[i];
        }
    }

    template <typename DEVICE, typename T, auto M, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(const T source[M][N], T target[M][N]) {
        for(typename DEVICE::index_t i = 0; i < M; i++) {
            for(typename DEVICE::index_t j = 0; j < N; j++) {
                target[i][j] = source[i][j];
            }
        }
    }

    template <typename DEVICE, typename T, auto M, auto N, auto P>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(const T source[M][N][P], T target[M][N][P]) {
        for(typename DEVICE::index_t i = 0; i < M; i++) {
            for(typename DEVICE::index_t j = 0; j < N; j++) {
                for(typename DEVICE::index_t k = 0; k < P; k++) {
                    target[i][j][k] = source[i][j][k];
                }
            }
        }
    }
    template <typename DEVICE, typename T, auto M, auto N, auto P>
    RL_TOOLS_FUNCTION_PLACEMENT void assign(const T source[P], T target[M][N][P]) {
        for(typename DEVICE::index_t i = 0; i < M; i++) {
            for(typename DEVICE::index_t j = 0; j < N; j++) {
                for(typename DEVICE::index_t k = 0; k < P; k++) {
                    target[i][j][k] = source[k];
                }
            }
        }
    }

    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT T norm(const T source[N]) {
        T acc = 0;
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            acc += source[i]*source[i];
        }
        typename DEVICE::SPEC::MATH math_dev;
        return math::sqrt(math_dev, acc);
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT void normalize(const T source[N], T target[N]) {
        T n = norm(source);
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            target[i] = source[i]/n;
        }
    }
    template <typename DEVICE, typename T, auto N>
    RL_TOOLS_FUNCTION_PLACEMENT T mean(const T source[N]) {
        T acc = 0;
        for(typename DEVICE::index_t i = 0; i < N; i++) {
            acc += source[i];
        }
        return acc/N;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif