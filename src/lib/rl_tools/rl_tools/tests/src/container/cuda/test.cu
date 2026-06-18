#define RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA
#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/containers/matrix/operations_cuda.h>

namespace rlt = rl_tools;

#include <gtest/gtest.h>

template <typename DEVICE_CPU, typename DEVICE_GPU, typename T, typename TI, TI ROWS, TI COLS>
void test(DEVICE_CPU& device_cpu, DEVICE_GPU& device_gpu){
    rlt::Matrix<rlt::matrix::Specification<T, TI, ROWS, COLS>> test_cpu, test_gpu;
    rlt::malloc(device_cpu, test_cpu);
    rlt::malloc(device_gpu, test_gpu);
    auto rng_gpu = rlt::random::default_engine(typename DEVICE_GPU::SPEC::RANDOM{}, 0);
    rlt::randn(device_gpu, test_gpu, rng_gpu);
    rlt::copy(device_gpu, device_cpu, test_gpu, test_cpu);
    T mean = rlt::mean(device_cpu, test_cpu);
    T std = rlt::std(device_cpu, test_cpu);
    std::cout << "mean: " << mean << ", std: " << std << "\n";
    if(ROWS >=50 && COLS >= 50){
        ASSERT_NEAR(mean, 0, 0.1);
        ASSERT_NEAR(std, 1, 0.1);
    }
}

TEST(RL_TOOLS_CONTAINER_CUDA, RANDN){
    using DEVICE_GPU = rlt::devices::DEVICE_FACTORY_CUDA<>;
    using DEVICE_CPU = rlt::devices::DEVICE_FACTORY<>;
    using T = float;
    using TI = typename DEVICE_GPU::index_t;
    DEVICE_CPU device_cpu;
    DEVICE_GPU device_gpu;
    rlt::init(device_gpu);

    test<DEVICE_CPU, DEVICE_GPU, T, TI, 200, 100>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 1, 100>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 100, 1>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 100, 100>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 101, 101>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 10, 10>(device_cpu, device_gpu);
    test<DEVICE_CPU, DEVICE_GPU, T, TI, 1, 2>(device_cpu, device_gpu);
}
