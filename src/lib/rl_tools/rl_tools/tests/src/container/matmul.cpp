#include <rl_tools/operations/cpu.h>
#include <gtest/gtest.h>
namespace rlt = rl_tools;
using DEVICE = rlt::devices::DefaultCPU;
using T = float;
using TI = typename DEVICE::index_t;



template <TI M, TI N, TI K>
void matmul(){
    DEVICE device;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    rlt::Matrix<rlt::matrix::Specification<T, TI, M, K, false>> A;
    rlt::Matrix<rlt::matrix::Specification<T, TI, K, N, false>> B;
    rlt::Matrix<rlt::matrix::Specification<T, TI, M, N, false>> C, C_target;
    rlt::randn(device, A, rng);
    rlt::randn(device, B, rng);
    rlt::multiply(device, A, B, C);

    for(TI i = 0; i < M; i++){
        for(TI j = 0; j < N; j++){
            T sum = 0;
            for(TI k = 0; k < K; k++){
                sum += rlt::get(A, i, k) * rlt::get(B, k, j);
            }
            rlt::set(C_target, i, j, sum);
        }
    }

    T abs_diff = rlt::abs_diff(device, C, C_target);
    std::cout << "abs_diff: " << abs_diff << std::endl;
    ASSERT_LT(abs_diff, 1e-6);
}

TEST(RL_TOOLS_CONTAINER_MATMUL, TEST){
    matmul<10, 10, 10>();
    matmul<1, 1, 1>();
    matmul<1, 1, 2>();
    matmul<1, 2, 1>();
    matmul<2, 1, 1>();
    matmul<2, 2, 1>();
    matmul<2, 2, 2>();
    matmul<10, 2, 2>();
    matmul<10, 10, 2>();
    matmul<10, 10, 10>();
    matmul<1, 10, 10>();
    matmul<32, 10, 10>();
}
