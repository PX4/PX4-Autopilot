#include "../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_DEVICES_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_DEVICES_CUDA_H

#include "../rl_tools.h"
#include "../utils/generic/typing.h"
#include "../containers/matrix/matrix.h"
#include "devices.h"
#include "cpu.h"
#include <cublas_v2.h>
#include <vector>
#include <unordered_map>
#include <curand_kernel.h>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::devices{
    namespace cuda{
        struct Base{
            static constexpr DeviceId DEVICE_ID = DeviceId::CUDA;
            using index_t = unsigned int;
            static constexpr index_t MAX_INDEX = -1;
        };
    }
    namespace math{
        struct CUDA: cuda::Base{
            static constexpr Type TYPE = Type::math;
        };
    }
    namespace random{
        struct CUDA:devices::random::Generic<devices::math::CUDA>, cuda::Base{
            static constexpr Type TYPE = Type::random;
            template <typename T_TI, T_TI T_NUM_RNGS, typename T_CURAND_TYPE=curandState>
            struct Specification{
                using TI = T_TI;
                static constexpr TI NUM_RNGS = T_NUM_RNGS;
                using CURAND_TYPE = T_CURAND_TYPE;
            };
            template <typename SPEC = Specification<cuda::Base::index_t, 1024>>
            struct ENGINE{
                using TI = typename SPEC::TI;
                static constexpr TI NUM_RNGS = SPEC::NUM_RNGS;
                Matrix<matrix::Specification<typename SPEC::CURAND_TYPE, TI, 1, NUM_RNGS, true>> states;
            };
        };
    }
    namespace logging{
        struct CUDA: cuda::Base{
            static constexpr Type TYPE = Type::logging;
        };
    }
    namespace cuda{
        template <typename T_SPEC>
        struct CUDA_FAT: cuda::Base{
            template <typename OTHER_DEVICE>
            static constexpr bool compatible = OTHER_DEVICE::DEVICE_ID == DeviceId::CUDA;
            using SPEC = T_SPEC;
            typename SPEC::LOGGING* logger = nullptr;
            cublasHandle_t handle;
            bool graph_capture_active = false;
            cudaStream_t stream;
#ifdef RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
            index_t malloc_counter = 0;
#endif
#ifdef RL_TOOLS_DEBUG_DEVICE_CUDA_CHECK_INIT
            bool initialized = false;
#endif
        };
    }
    // there is a "FAT" version (containing logger pointer and other context) and a "TAG" version that can be  just used for tag dispatch without runtime overhead
    template <typename T_SPEC>
    struct CUDA: utils::typing::conditional_t<T_SPEC::TAG, cuda::Base, cuda::CUDA_FAT<T_SPEC>>{
        using SPEC = T_SPEC;
        typename SPEC::MATH math;
        typename SPEC::RANDOM random;
        static constexpr bool TAG = SPEC::TAG;
        static constexpr bool KERNEL = SPEC::KERNEL;
    };
    namespace cuda{
//        template <typename DEVICE, typename std::enable_if<sizeof(CUDA<typename DEVICE::SPEC, true>) == 0>::type* = nullptr>
        template <typename SPEC, bool T_KERNEL=false>
        struct TAG_SPEC: SPEC{
            static constexpr bool TAG = true;
            static constexpr bool KERNEL = T_KERNEL;
        };
        template <typename DEVICE, bool KERNEL = false>
        using _TAG = CUDA<TAG_SPEC<typename DEVICE::SPEC, KERNEL>>;
#ifdef _MSC_VER
        template <typename DEVICE, bool KERNEL = false>
#else
        template <typename DEVICE, bool KERNEL = false, typename utils::typing::enable_if<sizeof(_TAG<DEVICE, KERNEL>) == 3>::type* = nullptr> // size three because C++ requires a size of at least one byte per struct (for distinct addresses) and since it has two empty member structs (random and math device structs)
#endif
        using TAG = _TAG<DEVICE, KERNEL>;

        struct CUDA_KERNEL_SPEC{
            struct RANDOM{
                template <typename T_ENGINE = curandState>
                using ENGINE = T_ENGINE;
            };
            using MATH = devices::math::CUDA;
            using LOGGING = logging::CUDA;
        };
        template <typename T_SPEC>
        struct CUDA_KERNEL{
            using SPEC = T_SPEC;
            using index_t = unsigned int;
            devices::random::CUDA random;
            typename SPEC::MATH math;
            typename SPEC::LOGGING* logger = nullptr;
        };

    }

    struct DefaultCUDASpecification{
//        using MATH_HOST = devices::math::CPU;
        using MATH = devices::math::CUDA;
        using MATH_DEVICE_ACCURATE = math::CUDA;
        using RANDOM = random::CUDA;
        using LOGGING = logging::CUDA;
        static constexpr bool TAG = false;
        static constexpr bool KERNEL = false;
    };
    using DefaultCUDA = CUDA<DefaultCUDASpecification>;
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#include <iostream>
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename SPEC>
    void init(devices::CUDA<SPEC>& device){
        cudaError_t stat;
        stat = cudaStreamCreate(&device.stream);
        if (stat != cudaSuccess) {
//            log(device.logger, (const char*)"CUBLAS initialization failed ", cublasGetStatusString(stat));
            std::cout << "CUDA Stream initialization failed " << cudaGetErrorString(stat) << std::endl;
        }
        cublasStatus_t cublas_stat;
        cublas_stat = cublasCreate(&device.handle);
        if (cublas_stat != CUBLAS_STATUS_SUCCESS) {
//            log(device.logger, (const char*)"CUBLAS initialization failed ", cublasGetStatusString(stat));
            std::cout << "CUBLAS initialization failed " << cublasGetStatusString(cublas_stat) << std::endl;
        }
        cublas_stat = cublasSetStream(device.handle, device.stream);
        if (cublas_stat != CUBLAS_STATUS_SUCCESS) {
//            log(device.logger, (const char*)"CUBLAS initialization failed ", cublasGetStatusString(stat));
            std::cout << "CUBLAS setting stream failed " << cublasGetStatusString(cublas_stat) << std::endl;
        }
#ifdef RL_TOOLS_DEBUG_DEVICE_CUDA_CHECK_INIT
        if(device.initialized){
            std::cerr << "CUDA device already initialized" << std::endl;
        }
        device.initialized = true;
#endif
    }
    template <typename SPEC>
    void check_status(devices::CUDA<SPEC>& device){
#ifdef RL_TOOLS_DEBUG_DEVICE_CUDA_CHECK_INIT
        if(!device.initialized){
            std::cerr << "CUDA device not initialized" << std::endl;
        }
#endif
#ifdef RL_TOOLS_DEBUG_DEVICE_CUDA_SYNCHRONIZE_STATUS_CHECK
        if (!device.graph_capture_active){
            cudaDeviceSynchronize();
            cudaStreamSynchronize(device.stream);
        }
        cudaError_t cudaStatus = cudaGetLastError();
        if (cudaStatus != cudaSuccess){
            std::string error_string = cudaGetErrorString(cudaStatus);
            std::cerr << "cuda failed: " << error_string << std::endl;
            std::exit(100);
        }
#endif
    }
    template <typename DEV_SPEC, typename TI>
    void count_malloc(devices::CUDA<DEV_SPEC>& device, TI size){
#ifdef RL_TOOLS_DEBUG_CONTAINER_COUNT_MALLOC
        device.malloc_counter += size;
#endif
    }

    void print_graph(cudaGraph_t graph){

        // Step 8: Retrieve and list nodes and dependencies
        // Get all nodes
        size_t numNodes = 0;
        cudaGraphGetNodes(graph, nullptr, &numNodes);
        std::vector<cudaGraphNode_t> nodes(numNodes);
        cudaGraphGetNodes(graph, nodes.data(), &numNodes);

        // Assign indices to nodes
        std::unordered_map<cudaGraphNode_t, int> nodeMap;
        for(int i = 0; i < numNodes; ++i){
            nodeMap[nodes[i]] = i;
        }

        // Get all edges (dependencies)
        size_t numEdges = 0;
        cudaGraphGetEdges(graph, nullptr, nullptr, &numEdges);
        std::vector<cudaGraphNode_t> srcNodes(numEdges);
        std::vector<cudaGraphNode_t> dstNodes(numEdges);
        if(numEdges > 0){
            cudaGraphGetEdges(graph, srcNodes.data(), dstNodes.data(), &numEdges);
        }

        // Map dependencies: destination node -> list of source nodes
        std::vector<std::vector<int>> dependencies(numNodes, std::vector<int>());
        for(int i = 0; i < numEdges; ++i){
            int src = nodeMap[srcNodes[i]];
            int dst = nodeMap[dstNodes[i]];
            dependencies[dst].push_back(src);
        }

        // Print nodes and their dependencies
        std::cout << "CUDA Graph Nodes and Dependencies:\n";
        for(int i = 0; i < numNodes; ++i){
            std::cout << "Node " << i << ": ";
            cudaGraphNodeType nodeType;
            cudaGraphNodeGetType(nodes[i], &nodeType);

            std::string typeName;
            switch(nodeType){
            case cudaGraphNodeTypeKernel:
                typeName = "Kernel";
                break;
            case cudaGraphNodeTypeMemcpy:
                typeName = "Memcpy";
                break;
            case cudaGraphNodeTypeMemset:
                typeName = "Memset";
                break;
                // Add other cases as needed
            default:
                typeName = "Other";
                break;
            }
            std::cout << typeName << " ";
            if(dependencies[i].empty()){
                std::cout << "No dependencies";
            }
            else{
                std::cout << "Depends on Node(s): ";
                for(auto &dep : dependencies[i]){
                    std::cout << dep << " ";
                }
            }
            std::cout << "\n";
        }
    }

}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
