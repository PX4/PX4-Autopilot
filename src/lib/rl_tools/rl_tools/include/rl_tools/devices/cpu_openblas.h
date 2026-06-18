#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CPU_OPENBLAS_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CPU_OPENBLAS_H

#include "../utils/generic/typing.h"
#include "devices.h"

#include "cpu_blas.h"

// based on: https://github.com/OpenMathLib/OpenBLAS/blob/24c5152fbd8027a39759acf261e1595326ab0114/cblas.h
extern "C" {
    typedef enum CBLAS_ORDER     {CblasRowMajor=101, CblasColMajor=102} CBLAS_ORDER;
    typedef enum CBLAS_TRANSPOSE {CblasNoTrans=111, CblasTrans=112, CblasConjTrans=113, CblasConjNoTrans=114} CBLAS_TRANSPOSE;
    #ifndef OPENBLAS_CONST
    # define OPENBLAS_CONST const
    #endif
    typedef int blasint;
    void cblas_sgemm(OPENBLAS_CONST enum CBLAS_ORDER Order, OPENBLAS_CONST enum CBLAS_TRANSPOSE TransA, OPENBLAS_CONST enum CBLAS_TRANSPOSE TransB, OPENBLAS_CONST blasint M, OPENBLAS_CONST blasint N, OPENBLAS_CONST blasint K, OPENBLAS_CONST float alpha, OPENBLAS_CONST float *A, OPENBLAS_CONST blasint lda, OPENBLAS_CONST float *B, OPENBLAS_CONST blasint ldb, OPENBLAS_CONST float beta, float *C, OPENBLAS_CONST blasint ldc);
    void cblas_dgemm(OPENBLAS_CONST enum CBLAS_ORDER Order, OPENBLAS_CONST enum CBLAS_TRANSPOSE TransA, OPENBLAS_CONST enum CBLAS_TRANSPOSE TransB, OPENBLAS_CONST blasint M, OPENBLAS_CONST blasint N, OPENBLAS_CONST blasint K, OPENBLAS_CONST double alpha, OPENBLAS_CONST double *A, OPENBLAS_CONST blasint lda, OPENBLAS_CONST double *B, OPENBLAS_CONST blasint ldb, OPENBLAS_CONST double beta, double *C, OPENBLAS_CONST blasint ldc);
}

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    template <typename T_SPEC>
    struct CPU_OPENBLAS: CPU_BLAS<T_SPEC>{
        static constexpr DeviceId DEVICE_ID = DeviceId::CPU_OPENBLAS;
    };
    using DefaultCPU_OPENBLAS = CPU_OPENBLAS<DefaultCPUSpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEV_SPEC>
    void init(devices::CPU_OPENBLAS<DEV_SPEC>& device){
        init(static_cast<devices::CPU_BLAS<DEV_SPEC>&>(device));
        using DEVICE = devices::CPU_OPENBLAS<DEV_SPEC>;
        const char *env_var_name = "OPENBLAS_NUM_THREADS";
        const char *value = getenv(env_var_name);
        bool warn = true;
        if (value != NULL) {
            char *endptr;
            typename DEVICE::index_t num_threads = strtol(value, &endptr, 10);
            if (*endptr == '\0') {
                warn = num_threads != 1;
            }
        }
        if(warn){
            std::cerr << "Warning: " << env_var_name << " is not set to 1. This may degrade performance." << std::endl;
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
