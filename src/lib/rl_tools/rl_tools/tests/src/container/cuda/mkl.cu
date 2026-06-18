#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/containers/matrix/operations_cuda.h>

namespace rlt = rl_tools;

#include <gtest/gtest.h>

template <typename DEVICE_CPU, typename DEVICE_GPU, typename T, typename TI, TI M, TI K, TI N, TI N_ITERATIONS>
void test(DEVICE_CPU& device_cpu, DEVICE_GPU& device_gpu, std::tuple<T, T>& durations){
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, K>> A_cpu, A_gpu;
    rlt::Matrix<rlt::matrix::Specification<T, TI, K, N>> B_cpu, B_gpu;
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, N>> C_cpu, C_gpu, C_gpu_cpu;
    rlt::malloc(device_cpu, A_cpu);
    rlt::malloc(device_gpu, A_gpu);
    rlt::malloc(device_cpu, B_cpu);
    rlt::malloc(device_gpu, B_gpu);
    rlt::malloc(device_cpu, C_cpu);
    rlt::malloc(device_gpu, C_gpu);
    rlt::malloc(device_cpu, C_gpu_cpu);
    auto rng_gpu = rlt::random::default_engine(typename DEVICE_GPU::SPEC::RANDOM{}, 0);
    auto rng_cpu = rlt::random::default_engine(typename DEVICE_CPU::SPEC::RANDOM{}, 0);
    rlt::randn(device_gpu, A_gpu, rng_gpu);
    rlt::check_status(device_gpu);
    rlt::randn(device_cpu, B_cpu, rng_cpu);
    rlt::copy(device_gpu, device_cpu, A_gpu, A_cpu);
    rlt::copy(device_cpu, device_gpu, B_cpu, B_gpu);
    rlt::check_status(device_gpu);

    auto before_device_1_multiply = std::chrono::high_resolution_clock::now();
    for(TI i = 0; i < N_ITERATIONS; i++) {
        rlt::multiply(device_cpu, A_cpu, B_cpu, C_cpu);
    }
    auto after_device_1_multiply = std::chrono::high_resolution_clock::now();
    auto duration_device_1_multiply = std::chrono::duration_cast<std::chrono::microseconds>(after_device_1_multiply - before_device_1_multiply).count();

    auto before_device_2_multiply = std::chrono::high_resolution_clock::now();
    for(TI i = 0; i < N_ITERATIONS; i++) {
        rlt::multiply(device_gpu, A_gpu, B_gpu, C_gpu);
    }
    auto after_device_2_multiply = std::chrono::high_resolution_clock::now();
    auto duration_device_2_multiply = std::chrono::duration_cast<std::chrono::microseconds>(after_device_2_multiply - before_device_2_multiply).count();

    rlt::check_status(device_gpu);
    rlt::copy(device_gpu, device_cpu, C_gpu, C_gpu_cpu);
    rlt::check_status(device_gpu);

    std::cout << "A (" << M << "x" << K << "), B (" << K << "x" << N << "), C (" << M << "x" << N << "):" << std::endl;
    std::cout << "target:" << std::endl;
    rlt::print(device_cpu, C_cpu);
    std::cout << "real:" << std::endl;
    rlt::print(device_cpu, C_gpu_cpu);
    T diff = rlt::abs_diff(device_cpu, C_cpu, C_gpu_cpu);
    T diff_per_element = diff / (M * N);
    std::cout << "Matrix mul diff: " << diff << " per element: " << diff_per_element << std::endl;

    if(rlt::utils::typing::is_same_v<T, float>){
        ASSERT_TRUE(diff_per_element < 1e-5);
    }else{
        ASSERT_TRUE(diff_per_element < 1e-10);
    }
    durations = std::tuple<T, T>(duration_device_1_multiply, duration_device_2_multiply);
}

TEST(RL_TOOLS_CONTAINER_CUDA_MKL, GEMM){
    using DEVICE_GPU = rlt::devices::DEVICE_FACTORY_CUDA<>;
    using DEVICE_MKL = rlt::devices::DEVICE_FACTORY<>;
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using T = float;
    using TI = typename DEVICE_GPU::index_t;
    DEVICE_CPU device_cpu;
    DEVICE_GPU device_gpu;
    DEVICE_MKL device_mkl;
    rlt::init(device_gpu);

    constexpr TI N_ITERATIONS = 100;

    std::tuple<T, T> durations;

    test<DEVICE_CPU, DEVICE_GPU, float, TI, 5, 200, 100, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 45, 1, 100, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 35, 100, 1, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 25, 100, 100, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 5, 101, 101, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 15, 10, 10, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 5, 1, 1, N_ITERATIONS>(device_cpu, device_gpu, durations);
    test<DEVICE_CPU, DEVICE_GPU, float, TI, 1, 1, 1, N_ITERATIONS>(device_cpu, device_gpu, durations);

    test<DEVICE_MKL, DEVICE_GPU, float, TI, 5, 200, 100, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 45, 1, 100, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 35, 100, 1, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 25, 100, 100, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 5, 101, 101, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 15, 10, 10, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 5, 1, 1, N_ITERATIONS>(device_mkl, device_gpu, durations);
    test<DEVICE_MKL, DEVICE_GPU, float, TI, 1, 1, 1, N_ITERATIONS>(device_mkl, device_gpu, durations);

    test<DEVICE_MKL, DEVICE_GPU, float, TI, 128, 128, 128, N_ITERATIONS>(device_mkl, device_gpu, durations);
    std::cout << "CPU: " << std::get<0>(durations) << " us, GPU: " << std::get<1>(durations) << " us" << std::endl;
}
