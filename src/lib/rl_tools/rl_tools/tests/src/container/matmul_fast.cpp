
#define RL_TOOLS_DEVICES_DISABLE_REDEFINITION_DETECTION

#include <thread>
#include <rl_tools/operations/cpu.h>
#include <rl_tools/operations/cpu_mux.h>
namespace rlt = rl_tools;
using DEVICE = rlt::devices::DefaultCPU;
using DEVICE_MKL = rlt::devices::DEVICE_FACTORY<>;
using T = float;
using TI = typename DEVICE::index_t;

// #include <immintrin.h> // For AVX intrinsics

#include <chrono>

template <TI ITERATIONS, typename DEVICE, typename SPEC_A, typename SPEC_B, typename SPEC_C>
void benchmark(DEVICE& device, rlt::Matrix<SPEC_A>& A, rlt::Matrix<SPEC_B>& B, rlt::Matrix<SPEC_C>& C, std::string device_name, bool print = true){
    constexpr TI M = SPEC_C::ROWS;
    constexpr TI N = SPEC_C::COLS;
    constexpr TI K = SPEC_A::COLS;
    constexpr TI FLOPS = 2 * M * N * K * ITERATIONS;
    auto now = std::chrono::high_resolution_clock::now();
    for(TI i = 0; i < ITERATIONS; i++){
        rlt::multiply(device, A, B, C);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - now;
    T checksum = 0;
    for(TI i = 0; i < M; ++i){
        for(TI j = 0; j < N; ++j){
            checksum += rlt::get(C, i, j);
        }
    }
    if(print){
        std::cout << "        Device: " << device_name << std::endl;
        std::cout << "            Checksum: " << checksum << std::endl;
        std::cout << "            Elapsed time: " << elapsed.count() << "s" << std::endl;
        std::cout << "            GFLOPS: " << FLOPS / elapsed.count() / 1e9 << std::endl;
    }
}

template <TI M, TI N, TI K, TI ITERATIONS, bool DYNAMIC_ALLOCATION>
void matmul(){
    DEVICE device;
    DEVICE_MKL device_mkl;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Matrix<rlt::matrix::Specification<T, TI, M, K, DYNAMIC_ALLOCATION>> A;
    rlt::Matrix<rlt::matrix::Specification<T, TI, K, N, DYNAMIC_ALLOCATION>> B;
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, N, DYNAMIC_ALLOCATION>> C, C_target;
    if constexpr(DYNAMIC_ALLOCATION){
        rlt::malloc(device, A);
        rlt::malloc(device, B);
        rlt::malloc(device, C);
        rlt::utils::assert_exit(device, reinterpret_cast<TI>(A._data) % 64 == 0, "A._data % 64 == 0");
        rlt::utils::assert_exit(device, reinterpret_cast<TI>(B._data) % 64 == 0, "B._data % 64 == 0");
        rlt::utils::assert_exit(device, reinterpret_cast<TI>(C._data) % 64 == 0, "C._data % 64 == 0");
    }
    rlt::randn(device, A, rng);
    rlt::randn(device, B, rng);
    std::cout << "    M: " << M << " N: " << N << " K: " << K << std::endl;
    benchmark<20>(device_mkl, A, B, C, "mkl", false);
    benchmark<ITERATIONS>(device_mkl, A, B, C, "mkl");
    benchmark<20>(device, A, B, C, "generic", false);
    benchmark<ITERATIONS/10>(device, A, B, C, "generic");

}

int main(){
    // stack
    // std::cout << "Stack" << std::endl;
    // matmul<32, 32, 32, 1000000, false>();
    // matmul<64, 64, 64, 100000, false>();
    // matmul<128, 128, 128, 10000, false>();
    // heap
    std::cout << "Heap" << std::endl;
    matmul<32, 32, 32, 1000000, true>();
    matmul<64, 64, 64, 100000, true>();
    matmul<128, 128, 128, 10000, true>();
    // matmul<256, 256, 256, 1000, true>();
    // matmul<512, 512, 512, 100, true>();
    // matmul<1024, 1024, 1024, 10, true>();
    return 0;
}
