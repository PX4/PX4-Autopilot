#define RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION
#define RL_TOOLS_FUNCTION_PLACEMENT __host__ __device__

#include <rl_tools/operations/cpu.h>

#include <rl_tools/operations/cuda.h>


#include <rl_tools/rl/environments/l2f/operations_cpu.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <iostream>
#include <chrono>
#include <cassert>

using T = float;
constexpr size_t N_BLOCKS = 4096;
constexpr size_t N_BLOCKS_CPU = 1;
constexpr size_t N_THREADS = 512;
constexpr size_t N_THREADS_CPU = 1;
constexpr size_t N_ITERATIONS = 1000000;

using DEVICE_GPU = rlt::devices::CUDA<rlt::devices::DefaultCUDASpecification>;
using DEVICE_CPU = rlt::devices::CPU<rlt::devices::DefaultCPUSpecification>;

using TI_GPU = DEVICE_CPU::index_t;
using TI_CPU = DEVICE_CPU::index_t;


// to specify the dynamics of the quadrotor that should be simulated (this includes rotor dynamics)
namespace builder {
    using namespace rlt::rl::environments::l2f;
    using namespace rlt::rl::environments::l2f::observation;
    struct ENVIRONMENT_STATIC_PARAMETERS{
        static constexpr TI_GPU N_SUBSTEPS = 1;
        static constexpr TI_GPU ACTION_HISTORY_LENGTH = 16;
        static constexpr TI_GPU CLOSED_FORM = false;
        using STATE_BASE = StateBase<StateSpecification<T, TI_GPU>>;
        using STATE_TYPE = StateRotors<StateRotorsSpecification<T, TI_GPU, CLOSED_FORM, StateRandomForce<StateSpecification<T, TI_GPU, STATE_BASE>>>>;
        using OBSERVATION_TYPE =
            Position<PositionSpecification<T, TI_GPU,
            OrientationRotationMatrix<OrientationRotationMatrixSpecification<T, TI_GPU,
            LinearVelocity<LinearVelocitySpecification<T, TI_GPU,
            AngularVelocity<AngularVelocitySpecification<T, TI_GPU>>>>>>>>;
        using OBSERVATION_TYPE_PRIVILEGED =
            Position<PositionSpecificationPrivileged<T, TI_GPU,
            OrientationRotationMatrix<OrientationRotationMatrixSpecificationPrivileged<T, TI_GPU,
            LinearVelocity<LinearVelocitySpecificationPrivileged<T, TI_GPU,
            AngularVelocity<AngularVelocitySpecificationPrivileged<T, TI_GPU,
            RandomForce<RandomForceSpecification<T, TI_GPU,
            RotorSpeeds<RotorSpeedsSpecification<T, TI_GPU>>>>>>>>>>>>;
        static constexpr bool PRIVILEGED_OBSERVATION_NOISE = false;
        using DEFAULT_STUB = parameters::DEFAULT_PARAMETERS_FACTORY<T, TI_GPU>;
        using PARAMETERS = DEFAULT_STUB::PARAMETERS_TYPE;
        static constexpr auto PARAMETER_VALUES = DEFAULT_STUB::nominal_parameters;
        static constexpr T STATE_LIMIT_POSITION = 100000;
        static constexpr T STATE_LIMIT_VELOCITY = 100000;
        static constexpr T STATE_LIMIT_ANGULAR_VELOCITY = 100000;
    };
}

using ENVIRONMENT_SPEC = rl_tools::rl::environments::l2f::Specification<T, TI_GPU, builder::ENVIRONMENT_STATIC_PARAMETERS>;
using ENVIRONMENT = rl_tools::rl::environments::Multirotor<ENVIRONMENT_SPEC>;


template <TI_GPU T_N_BLOCKS, TI_GPU T_BLOCK_DIM, TI_GPU T_N_ITERATIONS>
struct SimulateParallelSpec{
    static constexpr TI_GPU N_BLOCKS = T_N_BLOCKS;
    static constexpr TI_GPU BLOCK_DIM = T_BLOCK_DIM;
    static constexpr TI_GPU N_ITERATIONS = T_N_ITERATIONS;
};

template <typename DEVICE, typename SPEC_SIMULATE, typename RNG>
void simulate_sequential(DEVICE& device, const ENVIRONMENT* envs, ENVIRONMENT::Parameters* parameters, ENVIRONMENT::State* states_input, typename ENVIRONMENT::State* next_states_output, const SPEC_SIMULATE, RNG& rng) {
    using STATE = typename ENVIRONMENT::State;
    using TI = typename DEVICE::index_t;
    for(TI block_i = 0; block_i < SPEC_SIMULATE::N_BLOCKS; block_i++){
        for(TI thread_i = 0; thread_i < SPEC_SIMULATE::BLOCK_DIM; thread_i++){
            const TI full_id = block_i * SPEC_SIMULATE::BLOCK_DIM + thread_i;
            const auto& this_env = envs[full_id];
            auto& this_parameters = parameters[full_id];
            STATE state;
            STATE next_state;
            state = states_input[full_id];
            for(TI iteration_i=0; iteration_i<SPEC_SIMULATE::N_ITERATIONS; iteration_i++){
                rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM, false>> action;
                for(TI action_i=0; action_i<ENVIRONMENT::ACTION_DIM; action_i++){
                    rlt::set(action, 0, 0, 0);
                }
                rlt::step(device, this_env, this_parameters, state, action, next_state, rng);
                state = next_state;
            }
            next_states_output[full_id] = state;
        }
    }
}

template <typename DEVICE, typename SPEC_SIMULATE>
__global__ void
__launch_bounds__(SPEC_SIMULATE::BLOCK_DIM)//, minBlocksPerMultiprocessor, maxBlocksPerCluster)
simulate_parallel(DEVICE& device, const ENVIRONMENT* envs, ENVIRONMENT::Parameters* parameters, const typename ENVIRONMENT::State* states_input, typename ENVIRONMENT::State* next_states_output, const SPEC_SIMULATE) {
    using STATE = typename ENVIRONMENT::State;
    using TI = typename DEVICE::index_t;
    const TI full_id = blockIdx.x * blockDim.x + threadIdx.x;
    const TI thread_id = threadIdx.x;
    const TI block_id = blockIdx.x;
    __shared__ ENVIRONMENT this_env;
    __shared__ ENVIRONMENT::Parameters this_parameters;
    if(thread_id == 0){
        this_env = envs[block_id];
        this_parameters = parameters[block_id * SPEC_SIMULATE::BLOCK_DIM];
    }
    __syncthreads();
    STATE state;
    STATE next_state;
    curandState rng;
    curand_init(0, full_id, 0, &rng);
    state = states_input[full_id];
    for(TI iteration_i=0; iteration_i<SPEC_SIMULATE::N_ITERATIONS; iteration_i++){
        rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM, false>> action;
        for(TI action_i=0; action_i<ENVIRONMENT::ACTION_DIM; action_i++){
            rlt::set(action, 0, 0, 0);
        }
        rlt::step(device, this_env, this_parameters, state, action, next_state, rng);
        state = next_state;
    }
    next_states_output[full_id] = state;
}

int main(void) {

    std::cout << "sizeof(ENVIRONMENT) = " << sizeof(ENVIRONMENT) << std::endl;
    std::cout << "sizeof(ENVIRONMENT::Parameters) = " << sizeof(ENVIRONMENT::Parameters) << std::endl;
    std::cout << "sizeof(ENVIRONMENT::State) = " << sizeof(ENVIRONMENT::State) << std::endl;

    int nDevices;

    cudaGetDeviceCount(&nDevices);
    for (int i = 0; i < nDevices; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        std::cout << "Device Number: " << i << std::endl;
        std::cout << "\tName: " << prop.name << std::endl;
        std::cout << "\tCompute capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "\tNumber of SMs: " << prop.multiProcessorCount << std::endl;
        std::cout << "\tRegisters per Multiprocessor: " << prop.regsPerMultiprocessor << std::endl;
        std::cout << "\tMax threads per SM: " << prop.maxThreadsPerMultiProcessor << std::endl;
    }
    TI_CPU chosen_device = 0;
    cudaSetDevice(chosen_device);
    cudaDeviceProp props;
    cudaGetDeviceProperties(&props, chosen_device);


    DEVICE_CPU device_cpu;

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device_cpu, rng);
    rlt::init(device_cpu, rng, 0);


    ENVIRONMENT* envs_cpu = (ENVIRONMENT*)malloc(sizeof(ENVIRONMENT) * N_BLOCKS * N_THREADS);
    ENVIRONMENT::Parameters* parameters_cpu = (ENVIRONMENT::Parameters*)malloc(sizeof(ENVIRONMENT::Parameters) * N_BLOCKS * N_THREADS);
    ENVIRONMENT::State* initial_states_cpu = (ENVIRONMENT::State*)malloc(sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS);
    ENVIRONMENT::State* final_states_cpu = (ENVIRONMENT::State*)malloc(sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS);
    ENVIRONMENT::State* final_states_gpu_cpu = (ENVIRONMENT::State*)malloc(sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS);

    for(TI_CPU block_i=0; block_i<N_BLOCKS; block_i++){
        for(TI_CPU thread_i=0; thread_i<N_THREADS; thread_i++){
            auto& this_env = envs_cpu[block_i * N_THREADS + thread_i];
            auto& this_parameters = parameters_cpu[block_i * N_THREADS + thread_i];
            auto& this_initial_state = initial_states_cpu[block_i * N_THREADS + thread_i];
            rlt::initial_parameters(device_cpu, this_env, this_parameters);
            rlt::initial_state(device_cpu, this_env, this_parameters, this_initial_state);
        }
    }

    {
        auto start = std::chrono::high_resolution_clock::now();
        simulate_sequential(device_cpu, envs_cpu, parameters_cpu, initial_states_cpu, final_states_cpu, SimulateParallelSpec<N_BLOCKS_CPU, N_THREADS_CPU, N_ITERATIONS>{}, rng);
        auto end = std::chrono::high_resolution_clock::now();

        double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "Simulation time (CPU):  " << elapsedTime << " ms (" << N_BLOCKS_CPU * N_THREADS_CPU * N_ITERATIONS / (elapsedTime / 1000.0) / 1e6 << " Msteps/s)" << std::endl;
    }

    DEVICE_GPU device_gpu;
    {
        ENVIRONMENT* envs_gpu;
        ENVIRONMENT::Parameters* parameters_gpu;
        ENVIRONMENT::State *initial_states_gpu, *final_states_gpu;

        cudaMalloc((void **)&envs_gpu, N_BLOCKS * sizeof(ENVIRONMENT));
        cudaMemcpy(envs_gpu, envs_cpu, N_BLOCKS * sizeof(ENVIRONMENT), cudaMemcpyHostToDevice);

        cudaMalloc((void**)&parameters_gpu, N_BLOCKS * N_THREADS * sizeof(ENVIRONMENT::Parameters));
        cudaMemcpy(parameters_gpu, parameters_cpu, N_BLOCKS * N_THREADS * sizeof(ENVIRONMENT::Parameters), cudaMemcpyHostToDevice);

        cudaMalloc((void **)&initial_states_gpu, sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS);
        cudaMemcpy(initial_states_gpu, initial_states_cpu,  sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS, cudaMemcpyHostToDevice);

        cudaMalloc((void **)&final_states_gpu,  sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS);

        cudaEvent_t start, stop;
        cudaEventCreate(&start);
        cudaEventCreate(&stop);
        cudaEventRecord(start, 0);
        dim3 grid(N_BLOCKS);
        dim3 threadsPerBlock(N_THREADS);
        simulate_parallel<<<grid, threadsPerBlock>>>(device_gpu, envs_gpu, parameters_gpu, initial_states_gpu, final_states_gpu, SimulateParallelSpec<N_BLOCKS, N_THREADS, N_ITERATIONS>{});
        cudaEventRecord(stop, 0);
        cudaEventSynchronize(stop);
        auto err = cudaGetLastError();
        if (cudaSuccess != err) {
            fprintf(stderr, "cudaCheckError() failed : %s\n", cudaGetErrorString(err));
        }
        float elapsedTime;
        cudaEventElapsedTime(&elapsedTime, start, stop);
        cudaEventDestroy(start);
        cudaEventDestroy(stop);
        cudaDeviceSynchronize();
        cudaMemcpy(final_states_gpu_cpu, final_states_gpu,  sizeof(ENVIRONMENT::State) * N_BLOCKS * N_THREADS, cudaMemcpyDeviceToHost);
        cudaFree(envs_gpu); cudaFree(parameters_gpu); cudaFree(initial_states_gpu); cudaFree(final_states_gpu);
        cudaDeviceSynchronize();

        std::cout << "Simluation dt: " << parameters_cpu->integration.dt << std::endl;
        std::cout << "Simulation time (GPU):  " << elapsedTime << " ms (" << N_BLOCKS * N_THREADS * N_ITERATIONS / (elapsedTime / 1000.0) / 1e6 << " Msteps/s, " << N_BLOCKS * N_THREADS * N_ITERATIONS * parameters_cpu->integration.dt / (elapsedTime / 1000.0) / (365 * 24 * 3600) << " Years/s)" << std::endl;
    }


    return 0;
}