#define RL_TOOLS_DISABLE_DYNAMIC_MEMORY_ALLOCATIONS
#include <rl_tools/operations/cuda.h>

#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/rl/environment_wrappers/scale_observations/operations_generic.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_generic.h>
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>



#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

#ifdef RL_TOOLS_ENABLE_JSON
#include <nlohmann/json.hpp>
#include <fstream>
#endif

namespace rlt = rl_tools;

#include "../cpu/config.h"

using DEVICE = rlt::devices::DefaultCUDA;
using KERNEL_DEVICE = rlt::devices::cuda::CUDA_KERNEL<rlt::devices::cuda::CUDA_KERNEL_SPEC>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<float>;
using TI = typename DEVICE::index_t;
static constexpr bool DYNAMIC_ALLOCATION = false;

using CONFIG = CONFIG_FACTORY<KERNEL_DEVICE, TYPE_POLICY, DYNAMIC_ALLOCATION>;

using LOOP_CONFIG = CONFIG::LOOP_CORE_CONFIG;
using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;


template <typename DEVICE>
__global__ void init(DEVICE& device, LOOP_STATE* ts){
    if (threadIdx.x == 0){
        printf("init using thread %d\n", threadIdx.x);
        rlt::malloc(device, *ts);
        constexpr TI SEED = 1;
        rlt::init(device, *ts, SEED);
    }
}

template <typename DEVICE>
__global__ void step(DEVICE& device, LOOP_STATE* ts){
    if (threadIdx.x == 0){
        rlt::step(device, *ts);
    }
}


int main(int argc, char** argv) {
    KERNEL_DEVICE device;
    LOOP_STATE* ts_cpu = (LOOP_STATE*)malloc(sizeof(LOOP_STATE));
    LOOP_STATE* ts = nullptr;
    std::cout << "Allocating " << sizeof(LOOP_STATE) << " bytes" << std::endl;
    auto error = cudaMalloc(&ts, sizeof(LOOP_STATE));
    if (error != cudaSuccess){
        std::cerr << "cudaMalloc failed: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    std::cout << "Launching kernel..." << std::endl;
    init<<<1, 1>>>(device, ts);
    error = cudaGetLastError();
    if (error != cudaSuccess){
        std::cerr << "Kernel launch failed: " << cudaGetErrorString(error) << std::endl;
        cudaFree(ts);
        return 1;
    }
    std::cout << "Synchronizing..." << std::endl;
    error = cudaDeviceSynchronize();
    if (error != cudaSuccess){
        std::cerr << "Kernel execution failed: " << cudaGetErrorString(error) << std::endl;
        cudaFree(ts);
        return 1;
    }
    std::cout << "Kernel completed successfully" << std::endl;
    cudaMemcpy(ts_cpu, ts, sizeof(LOOP_STATE), cudaMemcpyDeviceToHost);
    for (TI step_i=0; step_i < 700; step_i++){
        step<<<1, 1>>>(device, ts);
        error = cudaGetLastError();
        if (error != cudaSuccess){
            std::cerr << "Kernel launch failed: " << cudaGetErrorString(error) << std::endl;
            cudaFree(ts);
            return 1;
        }
        error = cudaDeviceSynchronize();
        if (error != cudaSuccess){
            std::cerr << "Kernel execution failed: " << cudaGetErrorString(error) << std::endl;
            cudaFree(ts);
            return 1;
        }
    }
    cudaMemcpy(ts_cpu, ts, sizeof(LOOP_STATE), cudaMemcpyDeviceToHost);

    cudaFree(ts);
    return 0;
}

// Should take ~ 0.3s on M3 Pro in BECHMARK mode
// - tested @ 1118e19f904a26a9619fac7b1680643a0afcb695)
// - tested @ 361c2f5e9b14d52ee497139a3b82867fce0404a7
