#include <cuda_bf16.h>
#include <cuda_runtime.h>
#include <mma.h>
#include <stdio.h>

using namespace nvcuda;

__global__ void init_kernel(__nv_bfloat16* global_a, __nv_bfloat16* global_b, float* global_c, int M, int N, int K) {
    if (threadIdx.x == 0) {
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < K; j++) {
                global_a[i * K + j] = __nv_bfloat16(i * K + j);
                global_b[i * N + j] = __nv_bfloat16(j * N + i);
                global_c[i * N + j] = 0.0f;
            }
        }
    }
}

__global__ void wmma_kernel(__nv_bfloat16* global_a, __nv_bfloat16* global_b, float* global_c, int M, int N, int K) {
    __shared__ __nv_bfloat162 shared_a_mem[16][8];
    __shared__ __nv_bfloat162 shared_b_mem[16][8];

    int loads_per_row = 8;
    int lane_id = threadIdx.x % loads_per_row;
    int row_id = threadIdx.x / loads_per_row;
    int increment = 32 / loads_per_row;
    for (int i = 0; i < 16; i += increment) {
        int index = (i + row_id) * K + lane_id * 2;
        shared_a_mem[i + row_id][lane_id] = *reinterpret_cast<__nv_bfloat162*>(&global_a[index]);
        shared_b_mem[i + row_id][lane_id] = *reinterpret_cast<__nv_bfloat162*>(&global_b[index]);
    }

    __syncthreads();

    wmma::fragment<wmma::matrix_a, 16, 16, 16, __nv_bfloat16, wmma::row_major> a_frag;
    wmma::fragment<wmma::matrix_b, 16, 16, 16, __nv_bfloat16, wmma::col_major> b_frag; // B is column-major
    wmma::fragment<wmma::accumulator, 16, 16, 16, float> c_frag;

    __nv_bfloat16* shared_a = reinterpret_cast<__nv_bfloat16*>(shared_a_mem);
    __nv_bfloat16* shared_b = reinterpret_cast<__nv_bfloat16*>(shared_b_mem);
    wmma::load_matrix_sync(a_frag, shared_a, 16);
    wmma::load_matrix_sync(b_frag, shared_b, 16);

    wmma::fill_fragment(c_frag, 0.0f);
    wmma::mma_sync(c_frag, a_frag, b_frag, c_frag);
    wmma::store_matrix_sync(&global_c[0], c_frag, N, wmma::mem_row_major);
}

int main() {
    int M = 16;
    int N = 16;
    int K = 16;

    __nv_bfloat16 *a, *b;
    float *c;
    cudaMalloc(&a, M * K * sizeof(__nv_bfloat16));
    cudaMalloc(&b, K * N * sizeof(__nv_bfloat16));
    cudaMalloc(&c, M * N * sizeof(float));

    __nv_bfloat16* b_cpu = static_cast<__nv_bfloat16*>(malloc(K * N * sizeof(__nv_bfloat16)));
    float* c_cpu = static_cast<float*>(malloc(M * N * sizeof(float)));

    init_kernel<<<1, 1>>>(a, b, c, M, N, K);
    cudaDeviceSynchronize();

    wmma_kernel<<<1, 32>>>(a, b, c, M, N, K);
    cudaDeviceSynchronize();

    cudaMemcpy(c_cpu, c, M * N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(b_cpu, b, K * N * sizeof(__nv_bfloat16), cudaMemcpyDeviceToHost);

    // Print matrix B
    printf("Matrix B:\n");
    for (int i = 0; i < K; i++) {
        for (int j = 0; j < N; j++) {
            printf("%f ", __bfloat162float(b_cpu[i * N + j]));
        }
        printf("\n");
    }

    // Print matrix C
    printf("\nMatrix C:\n");
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            printf("%f ", c_cpu[i * N + j]);
        }
        printf("\n");
    }

    cudaFree(a);
    cudaFree(b);
    cudaFree(c);
    free(b_cpu);
    free(c_cpu);
    return 0;
}

