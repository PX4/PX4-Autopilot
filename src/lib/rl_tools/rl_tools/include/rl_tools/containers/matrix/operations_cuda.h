#include "../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_CONTAINERS_MATRIX_OPERATIONS_CUDA_H

#include "matrix.h"
#include "../../devices/cuda.h"
#include "operations_generic.h"

#include <cuda_runtime.h>
#include <cuda.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
#ifndef RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
    template<typename DEV_SPEC, typename T, typename T_TI, T_TI SIZE_BYTES, bool T_CONST>
    void malloc(devices::CUDA<DEV_SPEC>& device, matrix::MatrixDynamic<T, T_TI, SIZE_BYTES, T_CONST>& matrix){
        /* for checking the pitch
        {
            size_t pitch;
            cudaMallocPitch(&matrix._data, &pitch, SPEC::COLS * sizeof(typename SPEC::T), SPEC::ROWS);

        }
        */
#ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC
        utils::assert_exit(device, matrix._data == nullptr, "Matrix is already allocated");
#endif
        T *temp = nullptr;
        // auto result = cudaMalloc(&temp, SIZE_BYTES);
        auto result = cudaMalloc(&temp, SIZE_BYTES);
// #ifdef RL_TOOLS_DEBUG_CONTAINER_CHECK_MALLOC
        if (result != cudaSuccess) {
            std::cerr << "Failed to allocate container: " << cudaGetErrorString(result) << " size: " << SIZE_BYTES << std::endl;
        }
        matrix._data = temp;
        check_status(device);
        count_malloc(device, SIZE_BYTES);

// #endif
    }
    template<typename DEV_SPEC, typename T, typename T_TI, T_TI SIZE_BYTES, bool T_CONST>
    void free(devices::CUDA<DEV_SPEC>& device, matrix::MatrixDynamic<T, T_TI, SIZE_BYTES, T_CONST>& matrix){
        cudaFree(matrix._data);
        check_status(device);
    }
#endif
    namespace containers::cuda::kernels {
        template<typename DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
        __global__ void
        copy(const Matrix<SOURCE_SPEC> source, Matrix<TARGET_SPEC> target) {
            static_assert(containers::check_structure<SOURCE_SPEC, TARGET_SPEC>);
            using T = typename TARGET_SPEC::T;
            using TI = typename DEVICE::index_t;

            TI col = blockIdx.x * blockDim.x + threadIdx.x;
            if(col < TARGET_SPEC::COLS){
                for(TI row = 0; row < TARGET_SPEC::ROWS; row++){
                    set(target, row, col, get(source, row, col));
                }
            }
        }
        template<typename DEVICE, typename SPEC, typename VALUE_T>
        __global__
        void set_all(Matrix<SPEC> m, VALUE_T value){
            using TI = typename DEVICE::index_t;
            TI col = blockIdx.x * blockDim.x + threadIdx.x;
            if(col < SPEC::COLS){
                for(typename SPEC::TI row = 0; row < SPEC::ROWS; row++){
                    set(m, row, col, value);
                }
            }
        }
        template<typename DEV_SPEC, typename SPEC, typename RNG>
        __global__
        void randn(devices::CUDA<DEV_SPEC> device, Matrix<SPEC> m, typename SPEC::T mean, typename SPEC::T std, RNG rng){
            using DEVICE = devices::CUDA<DEV_SPEC>;
            using T = typename SPEC::T;
            using TI = typename DEVICE::index_t;
            TI col = blockIdx.x * blockDim.x + threadIdx.x;
            static_assert(SPEC::COLS <= RNG::NUM_RNGS, "Please increase the number of CUDA RNGs");
            auto& rng_state = get(rng.states, 0, col);
            if(col < SPEC::COLS){
                for(TI row = 0; row < SPEC::ROWS; row++){
                    T sample = random::normal_distribution::sample(typename DEVICE::SPEC::RANDOM{}, mean, std, rng_state);
                    set(m, row, col, sample);
                }
            }
        }
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy_layout_mismatch(devices::CUDA<SOURCE_DEV_SPEC>& source_device, devices::CUDA<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE = devices::CUDA<TARGET_DEV_SPEC>;
        static_assert(containers::check_structure<TARGET_SPEC, SOURCE_SPEC>);
//        static_assert(utils::typing::is_same_v<typename TARGET_SPEC::T, typename SOURCE_SPEC::T>);
        using SPEC = TARGET_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(TARGET_SPEC::COLS, BLOCKSIZE_COLS);
        dim3 grid(N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE_COLS);
        containers::cuda::kernels::copy<DEVICE, SOURCE_SPEC, TARGET_SPEC><<<grid, block, 0, source_device.stream>>>(source, target);
        check_status(target_device);
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CUDA<SOURCE_DEV_SPEC>& source_device, devices::CUDA<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE_CUDA = devices::CUDA<SOURCE_DEV_SPEC>;
        using SPEC = TARGET_SPEC;
        if constexpr(containers::check_memory_layout<TARGET_SPEC, SOURCE_SPEC>){
            cudaMemcpyAsync(target._data, source._data, SPEC::SIZE_BYTES, cudaMemcpyDeviceToDevice, source_device.stream);
            check_status(source_device);
        }
        else{
            copy_layout_mismatch(source_device, target_device, source, target);
        }
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy_layout_mismatch(devices::CPU<SOURCE_DEV_SPEC>& source_device, devices::CUDA<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE_CUDA = devices::CUDA<TARGET_DEV_SPEC>;
        static_assert(containers::check_structure<TARGET_SPEC, SOURCE_SPEC>);
        using SPEC = TARGET_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI ROW_PITCH = TARGET_SPEC::LAYOUT::template ROW_PITCH<TARGET_SPEC::ROWS, TARGET_SPEC::COLS>;
        constexpr TI COL_PITCH = TARGET_SPEC::LAYOUT::template COL_PITCH<TARGET_SPEC::ROWS, TARGET_SPEC::COLS>;
        constexpr bool TARGET_ROW_MAJOR = ROW_PITCH >= COL_PITCH;
        constexpr bool TARGET_DENSE = TARGET_ROW_MAJOR ? ROW_PITCH == TARGET_SPEC::COLS : COL_PITCH == TARGET_SPEC::ROWS;
        if constexpr(TARGET_DENSE){
            // make a temporary copy of the source matrix (with the same layout as the target) and then copy it directly
            Matrix<matrix::Specification<T, TI, SPEC::ROWS, SPEC::COLS, true, typename TARGET_SPEC::LAYOUT, false>> temp;
            using TEMP_SPEC = typename decltype(temp)::SPEC;
            static_assert(TEMP_SPEC::SIZE_BYTES == TARGET_SPEC::SIZE_BYTES);
            malloc(source_device, temp);
            copy(source_device, source_device, source, temp);
            auto temp_size = TEMP_SPEC::SIZE_BYTES;
            cudaMemcpyAsync(target._data, temp._data, temp_size, cudaMemcpyHostToDevice, target_device.stream);
            cudaStreamSynchronize(target_device.stream);
            check_status(target_device);
            free(source_device, temp);
        }
        else{
            // prevent large copy (if the target e.g. is a view)
            Matrix<matrix::Specification<T, TI, SPEC::ROWS, SPEC::COLS, true, typename SOURCE_SPEC::LAYOUT, false>> temp;
            malloc(target_device, temp);
            copy(source_device, target_device, source, temp);
            copy(target_device, target_device, temp, target);
        }
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CPU<SOURCE_DEV_SPEC>& source_device, devices::CUDA<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE_CUDA = devices::CUDA<SOURCE_DEV_SPEC>;
        using SPEC = TARGET_SPEC;
        if constexpr(containers::check_memory_layout<TARGET_SPEC, SOURCE_SPEC>){
            cudaMemcpyAsync(target._data, source._data, SPEC::SIZE_BYTES, cudaMemcpyHostToDevice, target_device.stream);
            cudaStreamSynchronize(target_device.stream);
            check_status(target_device);
        }
        else{
            copy_layout_mismatch(source_device, target_device, source, target);
        }
    }

    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy_layout_mismatch(devices::CUDA<SOURCE_DEV_SPEC>& source_device, devices::CPU<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE_CUDA = devices::CUDA<SOURCE_DEV_SPEC>;
        static_assert(containers::check_structure<TARGET_SPEC, SOURCE_SPEC>);
//        static_assert(utils::typing::is_same_v<typename TARGET_SPEC::T, typename SOURCE_SPEC::T>);
        using SPEC = TARGET_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        Matrix<matrix::Specification<T, TI, SPEC::ROWS, SPEC::COLS, true>> temp_gpu, temp_gpu2, temp_cpu;
        using TEMP_SPEC = typename decltype(temp_gpu)::SPEC;
        malloc(source_device, temp_gpu);
        copy(source_device, source_device, source, temp_gpu);
        malloc(target_device, temp_cpu);
        cudaMemcpyAsync(temp_cpu._data, temp_gpu._data, TEMP_SPEC::SIZE_BYTES, cudaMemcpyDeviceToHost, source_device.stream);
        cudaStreamSynchronize(source_device.stream);
        check_status(source_device);
        free(source_device, temp_gpu);
        copy(target_device, target_device, temp_cpu, target);
        free(target_device, temp_cpu);
    }
    template<typename SOURCE_DEV_SPEC, typename TARGET_DEV_SPEC, typename SOURCE_SPEC, typename TARGET_SPEC>
    void copy(devices::CUDA<SOURCE_DEV_SPEC>& source_device, devices::CPU<TARGET_DEV_SPEC>& target_device, const Matrix<SOURCE_SPEC>& source, Matrix<TARGET_SPEC>& target){
        using DEVICE_CUDA = devices::CUDA<SOURCE_DEV_SPEC>;
        using SPEC = TARGET_SPEC;
        if constexpr(containers::check_memory_layout<TARGET_SPEC, SOURCE_SPEC>){
            cudaMemcpyAsync(target._data, source._data, SPEC::SIZE_BYTES, cudaMemcpyDeviceToHost, source_device.stream);
            cudaStreamSynchronize(source_device.stream);
            check_status(target_device);
        }
        else{
            copy_layout_mismatch(source_device, target_device, source, target);
        }
    }

    template<typename DEV_SPEC, typename SPEC, typename VALUE_T>
    void set_all(devices::CUDA<DEV_SPEC>& device, Matrix<SPEC>& m, VALUE_T value){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::COLS, BLOCKSIZE_COLS);
        dim3 grid(N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE_COLS);
        containers::cuda::kernels::set_all<DEVICE, SPEC, VALUE_T><<<grid, block, 0, device.stream>>>(m, value);
        check_status(device);
    }
    template<typename DEV_SPEC, typename SPEC, typename RNG>
    void randn(devices::CUDA<DEV_SPEC>& device, Matrix<SPEC>& m, typename SPEC::T mean, typename SPEC::T std, RNG& rng){
        using DEVICE = devices::CUDA<DEV_SPEC>;
        using TI = typename DEVICE::index_t;
        constexpr TI BLOCKSIZE_COLS = 32;
        constexpr TI N_BLOCKS_COLS = RL_TOOLS_DEVICES_CUDA_CEIL(SPEC::COLS, BLOCKSIZE_COLS);
        dim3 grid(N_BLOCKS_COLS);
        dim3 block(BLOCKSIZE_COLS);
        devices::cuda::TAG<DEVICE, true> tag_device{};
        containers::cuda::kernels::randn<<<grid, block, 0, device.stream>>>(tag_device, m, mean, std, rng);
        check_status(device);
    }
    template<typename DEV_SPEC, typename SPEC, typename RNG>
    void randn(devices::CUDA<DEV_SPEC>& device, Matrix<SPEC>& m, RNG& rng){
        randn(device, m, 0, 1, rng);
    }
    namespace containers::matrix{
        template<bool ACCUMULATE, typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
        void multiply_blas(devices::CUDA<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output) {
            using DEVICE = devices::CUDA<DEV_SPEC>;
            static_assert(INPUT_SPEC_A::ROWS == OUTPUT_SPEC::ROWS);
            static_assert(INPUT_SPEC_A::COLS == INPUT_SPEC_B::ROWS);
            static_assert(INPUT_SPEC_B::COLS == OUTPUT_SPEC::COLS);
            static_assert(INPUT_SPEC_A::ROW_PITCH == 1 || INPUT_SPEC_A::COL_PITCH == 1); // dense row- or column-major
            static_assert(INPUT_SPEC_B::ROW_PITCH == 1 || INPUT_SPEC_B::COL_PITCH == 1); // dense row- or column-major

            using T = typename OUTPUT_SPEC::T;
            using TI = typename DEVICE::index_t;

            constexpr bool A_ROW_MAJOR = INPUT_SPEC_A::ROW_PITCH >= INPUT_SPEC_A::COLS;
            constexpr bool B_ROW_MAJOR = INPUT_SPEC_B::ROW_PITCH >= INPUT_SPEC_B::COLS;
            constexpr auto A_TRANSPOSE = A_ROW_MAJOR ? CUBLAS_OP_N : CUBLAS_OP_T;
            constexpr auto B_TRANSPOSE = B_ROW_MAJOR ? CUBLAS_OP_N : CUBLAS_OP_T;

            constexpr TI A_PITCH = A_ROW_MAJOR ? INPUT_SPEC_A::ROW_PITCH : INPUT_SPEC_A::COL_PITCH;
            constexpr TI B_PITCH = B_ROW_MAJOR ? INPUT_SPEC_B::ROW_PITCH : INPUT_SPEC_B::COL_PITCH;

            constexpr T alpha = 1;
            constexpr T beta = ACCUMULATE ? 1 : 0;
            constexpr auto m = OUTPUT_SPEC::ROWS;
            constexpr auto k = INPUT_SPEC_A::COLS;
            constexpr auto n = OUTPUT_SPEC::COLS;

            // NOTE: cuBLAS uses the column-major format
            // A is m x k
            // B is k x n
            // output is m x n
            // A^T is k x m
            // B^T is n x k
            // output^T is n x m


            cublasStatus_t stat;
            if constexpr(utils::typing::is_same_v<T, float>){
                stat = cublasSgemm(device.handle, B_TRANSPOSE, A_TRANSPOSE, n, m, k, &alpha, B._data, B_PITCH, A._data, A_PITCH,&beta, output._data, row_pitch(output));
            }
            else{
                stat = cublasDgemm(device.handle, B_TRANSPOSE, A_TRANSPOSE, n, m, k, &alpha, B._data, B_PITCH, A._data, A_PITCH,&beta, output._data, row_pitch(output));
            }
            if(stat != CUBLAS_STATUS_SUCCESS){
                std::cout << "CUBLAS ERROR: " << cublasGetStatusString(stat) << std::endl;
            }
        }
    }
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply(devices::CUDA<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        containers::matrix::multiply_blas<false>(device, A, B, output);
    }
    template<typename DEV_SPEC, typename INPUT_SPEC_A, typename INPUT_SPEC_B, typename OUTPUT_SPEC>
    void multiply_accumulate(devices::CUDA<DEV_SPEC>& device, const Matrix<INPUT_SPEC_A>& A, const Matrix<INPUT_SPEC_B>& B, Matrix<OUTPUT_SPEC>& output){
        containers::matrix::multiply_blas<true>(device, A, B, output);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif