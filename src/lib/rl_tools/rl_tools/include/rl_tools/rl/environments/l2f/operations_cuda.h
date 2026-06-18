#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_CUDA_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_OPERATIONS_CUDA_H

#include "operations_generic.h"

template <typename T, auto BLOCK_DIM, typename POLICY, auto N_ITERATIONS>
__global__ void
__launch_bounds__(N_THREADS)//, minBlocksPerMultiprocessor, maxBlocksPerCluster)
simulate_parallel(
        const Parameters<T, ACTION_DIM>* params_input,
        const POLICY* policies_input,
        const T* state_input, T* next_state_output) {
    const uint full_id = blockIdx.x * blockDim.x + threadIdx.x;
    const uint thread_id = threadIdx.x;
    const uint block_id = blockIdx.x;
    __shared__ Parameters<T, ACTION_DIM> params;
    __shared__ POLICY policy;
    if(thread_id == 0){
        params = params_input[block_id];
        policy = policies_input[block_id];
    }
    __syncthreads();
    T state[STATE_DIM];
    T next_state[STATE_DIM];
    memcpy(state,  &state_input[full_id *  STATE_DIM],  STATE_DIM * sizeof(T));
    for(typename DEVICE::index_t i=0; i<N_ITERATIONS; i++){
        T action[ACTION_DIM];
        evaluate(policy, state, action);
//        memcpy(state, action, ACTION_DIM * sizeof(T));
        next_state_rk4<T, ACTION_DIM>(params, state, action, params.dt, next_state);
        memcpy(state, next_state, STATE_DIM * sizeof(T));
//        printf("params %d.%d %f\n", block_id, thread_id, params.dt);
    }
    memcpy(&next_state_output[full_id * STATE_DIM], state, STATE_DIM * sizeof(T));
}
#endif