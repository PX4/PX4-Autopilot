// Group 1
#include <rl_tools/operations/cpu/group_1.h>
#include <rl_tools/operations/cuda/group_1.h>

// Group 2
#include <rl_tools/operations/cpu/group_2.h>
#include <rl_tools/operations/cuda/group_2.h>

// Group 3
#include <rl_tools/operations/cpu/group_3.h>
#include <rl_tools/operations/cuda/group_3.h>

#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_cuda.h>
#include <rl_tools/nn/operations_cuda.h>
#include <rl_tools/nn/loss_functions/mse/operations_cuda.h>
#include <rl_tools/nn_models/operations_generic.h>
#include <rl_tools/nn_models/operations_cpu.h>
#include <rl_tools/nn/optimizers/adam/operations_cuda.h>

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <gtest/gtest.h>

template <typename T, typename TI, TI DIM_1, TI DIM_2, TI OFFSET_1, TI OFFSET_2, TI ALIGNMENT_1, TI ALIGNMENT_2, TI DIM_3, TI DIM_4, TI OFFSET_3, TI OFFSET_4, TI ALIGNMENT_3, TI ALIGNMENT_4>
void COPY_CONTAINER() {
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    DEVICE_CUDA device_cuda;
    DEVICE_CPU device_cpu;
    rlt::init(device_cuda);

    {
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, DIM_1, DIM_2>> matrix_cuda;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu2;
        rlt::malloc(device_cpu, matrix_cpu);
        rlt::malloc(device_cuda, matrix_cuda);
        rlt::malloc(device_cpu, matrix_cpu2);

        rlt::set_all(device_cpu, matrix_cpu, 1337.0f);

        rlt::copy(device_cpu, device_cuda, matrix_cpu, matrix_cuda);
        rlt::copy(device_cuda, device_cpu, matrix_cuda, matrix_cpu2);
        auto diff = rlt::abs_diff(device_cpu, matrix_cpu, matrix_cpu2);
        ASSERT_FLOAT_EQ(diff, 0.0f);
        rlt::free(device_cpu, matrix_cpu);
        rlt::free(device_cuda, matrix_cuda);
        rlt::free(device_cpu, matrix_cpu2);
    }
    {
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2, true, rlt::matrix::layouts::RowMajorAlignment<DEVICE_CPU::index_t, ALIGNMENT_1>>> matrix_cpu;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, DIM_1, DIM_2>> matrix_cuda;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu2;
        rlt::malloc(device_cpu, matrix_cpu);
        rlt::malloc(device_cuda, matrix_cuda);
        rlt::malloc(device_cpu, matrix_cpu2);

        rlt::set_all(device_cpu, matrix_cpu, 1337.0f);

        rlt::copy(device_cpu, device_cuda, matrix_cpu, matrix_cuda);
        rlt::copy(device_cuda, device_cpu, matrix_cuda, matrix_cpu2);
        auto diff = rlt::abs_diff(device_cpu, matrix_cpu, matrix_cpu2);
        ASSERT_FLOAT_EQ(diff, 0.0f);
        rlt::free(device_cpu, matrix_cpu);
        rlt::free(device_cuda, matrix_cuda);
        rlt::free(device_cpu, matrix_cpu2);
    }

    {
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, DIM_1, DIM_2, true, rlt::matrix::layouts::RowMajorAlignment<DEVICE_CPU::index_t, ALIGNMENT_2>>> matrix_cuda;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu2;
        rlt::malloc(device_cpu, matrix_cpu);
        rlt::malloc(device_cuda, matrix_cuda);
        rlt::malloc(device_cpu, matrix_cpu2);

        rlt::set_all(device_cpu, matrix_cpu, 1337.0f);

        rlt::copy(device_cpu, device_cuda, matrix_cpu, matrix_cuda);
        static_assert(DIM_1 > OFFSET_1);
        static_assert(DIM_2 > OFFSET_2);
        increment(matrix_cpu, OFFSET_1, OFFSET_2, 17);
        rlt::copy(device_cuda, device_cpu, matrix_cuda, matrix_cpu2);
        auto diff = rlt::abs_diff(device_cpu, matrix_cpu, matrix_cpu2);
        ASSERT_FLOAT_EQ(diff, 17.0f);
        rlt::free(device_cpu, matrix_cpu);
        rlt::free(device_cuda, matrix_cuda);
        rlt::free(device_cpu, matrix_cpu2);
    }
    {
        static_assert(DIM_3 >= DIM_1);
        static_assert(DIM_4 >= DIM_2);
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu;
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, DIM_3, DIM_4, true, rlt::matrix::layouts::RowMajorAlignment<DEVICE_CPU::index_t, ALIGNMENT_3>>> matrix_cuda_data;
        static_assert((OFFSET_3 + DIM_1) < DIM_3);
        static_assert((OFFSET_4 + DIM_2) < DIM_4);
        rlt::malloc(device_cuda, matrix_cuda_data);
        auto matrix_cuda = rlt::view<DEVICE_CUDA, typename decltype(matrix_cuda_data)::SPEC, DIM_1, DIM_2>(device_cuda, matrix_cuda_data, OFFSET_3, OFFSET_4);
        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, DIM_1, DIM_2, true, rlt::matrix::layouts::RowMajorAlignment<DEVICE_CPU::index_t, ALIGNMENT_4>>> matrix_cuda2;

        rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, DIM_1, DIM_2>> matrix_cpu2;
        rlt::malloc(device_cpu, matrix_cpu);
        rlt::malloc(device_cuda, matrix_cuda2);
        rlt::malloc(device_cpu, matrix_cpu2);

        DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
        rlt::init(device_cpu, rng);

        for(DEVICE_CPU::index_t row_i = 0; row_i < decltype(matrix_cpu)::SPEC::ROWS; row_i++){
            for(DEVICE_CPU::index_t col_i = 0; col_i < decltype(matrix_cpu)::SPEC::COLS; col_i++){
                set(matrix_cpu, row_i, col_i, rlt::random::normal_distribution::sample(device_cpu.random, (T)0, (T)1, rng));
            }
        }

        rlt::copy(device_cpu, device_cuda, matrix_cpu, matrix_cuda);
        increment(matrix_cpu, OFFSET_1, OFFSET_2, 17);
        rlt::copy(device_cuda, device_cuda, matrix_cuda, matrix_cuda2);
        rlt::copy(device_cuda, device_cpu, matrix_cuda, matrix_cpu2);
        auto diff = rlt::abs_diff(device_cpu, matrix_cpu, matrix_cpu2);
        if (rlt::math::abs(device_cpu.math, diff - 17) > 1e-5){
            std::cout << "diff: " << diff << std::endl;
        }
        ASSERT_FLOAT_EQ(diff, 17.0f);
        rlt::free(device_cpu, matrix_cpu);
        rlt::free(device_cuda, matrix_cuda_data);
        rlt::free(device_cuda, matrix_cuda2);
        rlt::free(device_cpu, matrix_cpu2);
    }
}

TEST(RL_TOOLS_NN_CUDA, COPY_CONTAINER){
/*
template <typename T, typename TI, TI DIM_1, TI DIM_2, TI OFFSET_1, TI OFFSET_2, TI ALIGNMENT_1, TI ALIGNMENT_2, TI DIM_3, TI DIM_4, TI OFFSET_3, TI OFFSET_4, TI ALIGNMENT_3, TI ALIGNMENT_4>
    julia code to generate fuzzing calls
    s(dtype, dim_1, dim_2, alignment_1, alignment_2, dim_3, dim_4, alignment_3, alignment_4) = "COPY_CONTAINER<$dtype, unsigned int, $dim_1, $dim_2, $(rand(0:(dim_1-1))), $(rand(0:(dim_2-1))), $alignment_1, $alignment_2, $dim_3, $dim_4, $(rand(0:(dim_3-dim_1-1))), $(rand(0:(dim_4-dim_2-1))), $alignment_3, $alignment_4>();\n"
    t(dtype, dim_1, dim_2, alignment_1, alignment_2, alignment_3, alignment_4) = s(dtype, dim_1, dim_2, alignment_1, alignment_2, dim_1 + rand(0:1000), dim_2 + rand(0:1000), alignment_3, alignment_4)
    print(reduce((a,c)->a * t((rand(0:1) == 0 ? "float" : "double"), rand(1:1000), rand(1:1000), rand(1:1000), rand(1:1000), rand(1:1000), rand(1:1000)), 1:50, init=""))
*/
    COPY_CONTAINER<double, unsigned int, 889, 89, 205, 82, 866, 883, 1799, 671, 869, 508, 810, 202>();
    COPY_CONTAINER<double, unsigned int, 629, 554, 259, 244, 78, 57, 928, 1377, 158, 312, 363, 883>();
    COPY_CONTAINER<float, unsigned int, 857, 786, 790, 339, 266, 109, 1282, 1245, 93, 53, 431, 796>();
    COPY_CONTAINER<double, unsigned int, 822, 335, 659, 169, 279, 138, 953, 1230, 89, 654, 319, 616>();
    COPY_CONTAINER<double, unsigned int, 101, 561, 56, 344, 448, 865, 1089, 681, 661, 96, 836, 739>();
    COPY_CONTAINER<float, unsigned int, 827, 700, 428, 375, 367, 360, 831, 1160, 1, 261, 257, 829>();
    COPY_CONTAINER<float, unsigned int, 930, 73, 799, 23, 283, 388, 1904, 718, 886, 439, 477, 796>();
    COPY_CONTAINER<float, unsigned int, 298, 404, 297, 65, 906, 878, 828, 1389, 237, 465, 69, 893>();
    COPY_CONTAINER<double, unsigned int, 518, 235, 50, 145, 146, 997, 658, 1198, 118, 215, 687, 300>();
    COPY_CONTAINER<double, unsigned int, 652, 160, 31, 121, 795, 332, 1466, 163, 506, 1, 767, 433>();
    COPY_CONTAINER<double, unsigned int, 391, 673, 142, 288, 542, 190, 1077, 1337, 101, 141, 551, 492>();
    COPY_CONTAINER<double, unsigned int, 552, 679, 439, 655, 725, 128, 1307, 839, 53, 20, 929, 121>();
    COPY_CONTAINER<double, unsigned int, 374, 632, 27, 523, 799, 780, 651, 1448, 133, 356, 198, 832>();
    COPY_CONTAINER<float, unsigned int, 382, 371, 203, 234, 614, 561, 598, 602, 140, 80, 202, 226>();
    COPY_CONTAINER<double, unsigned int, 319, 926, 86, 29, 350, 678, 732, 1670, 350, 662, 765, 154>();
    COPY_CONTAINER<float, unsigned int, 947, 585, 115, 147, 275, 328, 1782, 962, 157, 256, 147, 782>();
    COPY_CONTAINER<double, unsigned int, 633, 460, 242, 302, 768, 134, 1155, 1056, 161, 236, 218, 969>();
    COPY_CONTAINER<float, unsigned int, 907, 103, 659, 29, 327, 396, 1456, 601, 256, 279, 646, 396>();
    COPY_CONTAINER<float, unsigned int, 725, 500, 77, 125, 204, 791, 1119, 1080, 159, 8, 224, 251>();
    COPY_CONTAINER<float, unsigned int, 58, 576, 0, 537, 1, 223, 166, 655, 27, 7, 23, 166>();
    COPY_CONTAINER<float, unsigned int, 112, 555, 1, 192, 764, 712, 470, 1354, 352, 592, 555, 589>();
    COPY_CONTAINER<double, unsigned int, 926, 750, 59, 378, 530, 481, 1625, 1647, 71, 175, 327, 302>();
    COPY_CONTAINER<float, unsigned int, 1000, 327, 310, 284, 472, 812, 1208, 686, 182, 287, 668, 28>();
    COPY_CONTAINER<double, unsigned int, 60, 630, 34, 496, 679, 815, 417, 1200, 143, 416, 156, 330>();
    COPY_CONTAINER<double, unsigned int, 949, 917, 446, 535, 940, 923, 1583, 1773, 301, 639, 630, 113>();
    COPY_CONTAINER<double, unsigned int, 188, 241, 111, 150, 452, 296, 676, 842, 72, 56, 207, 145>();
    COPY_CONTAINER<float, unsigned int, 712, 984, 394, 510, 281, 777, 1346, 1396, 236, 287, 269, 604>();
    COPY_CONTAINER<double, unsigned int, 883, 480, 208, 154, 275, 905, 1174, 1249, 57, 341, 630, 483>();
    COPY_CONTAINER<double, unsigned int, 633, 110, 550, 56, 799, 754, 1287, 406, 355, 144, 174, 214>();
    COPY_CONTAINER<double, unsigned int, 972, 654, 232, 120, 846, 167, 1899, 1307, 292, 463, 100, 270>();
    COPY_CONTAINER<float, unsigned int, 805, 242, 465, 135, 152, 833, 1764, 293, 811, 1, 217, 541>();
    COPY_CONTAINER<double, unsigned int, 221, 467, 92, 115, 865, 827, 369, 926, 108, 74, 710, 847>();
    COPY_CONTAINER<double, unsigned int, 845, 228, 245, 172, 616, 865, 1101, 258, 46, 22, 745, 617>();
    COPY_CONTAINER<double, unsigned int, 863, 184, 317, 150, 397, 524, 1201, 936, 126, 420, 878, 599>();
    COPY_CONTAINER<double, unsigned int, 786, 820, 408, 634, 22, 236, 1015, 1270, 217, 439, 122, 757>();
    COPY_CONTAINER<double, unsigned int, 635, 761, 292, 435, 377, 111, 715, 804, 39, 18, 679, 671>();
    COPY_CONTAINER<double, unsigned int, 417, 346, 102, 295, 359, 459, 1318, 451, 811, 84, 353, 309>();
    COPY_CONTAINER<double, unsigned int, 599, 30, 109, 6, 386, 624, 898, 680, 132, 3, 243, 914>();
    COPY_CONTAINER<double, unsigned int, 814, 201, 792, 20, 16, 492, 870, 360, 12, 25, 831, 807>();
    COPY_CONTAINER<float, unsigned int, 472, 269, 221, 190, 546, 672, 959, 1121, 235, 35, 201, 172>();
    COPY_CONTAINER<double, unsigned int, 754, 443, 21, 426, 131, 30, 1608, 837, 163, 361, 325, 783>();
    COPY_CONTAINER<double, unsigned int, 698, 320, 66, 278, 571, 404, 903, 941, 20, 92, 189, 286>();
    COPY_CONTAINER<double, unsigned int, 150, 415, 37, 101, 212, 665, 912, 525, 301, 98, 240, 332>();
    COPY_CONTAINER<double, unsigned int, 684, 36, 170, 17, 324, 247, 729, 576, 24, 0, 316, 947>();
    COPY_CONTAINER<double, unsigned int, 374, 76, 297, 38, 287, 407, 940, 458, 250, 100, 221, 595>();
    COPY_CONTAINER<double, unsigned int, 732, 855, 178, 538, 783, 148, 1264, 1071, 33, 81, 726, 481>();
    COPY_CONTAINER<double, unsigned int, 662, 556, 64, 426, 802, 636, 1292, 575, 594, 11, 152, 340>();
    COPY_CONTAINER<float, unsigned int, 824, 137, 298, 26, 65, 478, 1724, 613, 20, 79, 530, 291>();
    COPY_CONTAINER<float, unsigned int, 343, 788, 297, 413, 764, 630, 607, 1429, 190, 606, 408, 298>();
    COPY_CONTAINER<double, unsigned int, 759, 327, 312, 88, 485, 499, 1161, 692, 142, 97, 492, 507>();
}


TEST(RL_TOOLS_NN_CUDA, COPYING_VIEWS){
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    DEVICE_CPU device_cpu;
    using DTYPE = float;
    {

        DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
        rlt::init(device_cpu, rng);
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cpu_data;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cpu_data_2;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cpu_data_3;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cpu_data_3_orig;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cpu_data_4;
        rlt::Matrix<rlt::matrix::Specification<DTYPE, DEVICE_CPU::index_t, 100, 100>> matrix_cuda_data;
        rlt::malloc(device_cpu, matrix_cpu_data);
        rlt::malloc(device_cpu, matrix_cpu_data_2);
        rlt::malloc(device_cpu, matrix_cpu_data_3);
        rlt::malloc(device_cpu, matrix_cpu_data_3_orig);
        rlt::malloc(device_cpu, matrix_cpu_data_4);
        rlt::malloc(device_cuda, matrix_cuda_data);

        auto matrix_cpu_view = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data)::SPEC, 50, 50>(device_cuda, matrix_cpu_data, 25, 25);
        auto matrix_cpu_view_2 = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data_2)::SPEC, 50, 50>(device_cuda, matrix_cpu_data_2, 25, 25);
        auto matrix_cpu_view_3 = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data_3)::SPEC, 50, 50>(device_cuda, matrix_cpu_data_3, 25, 25);
        auto matrix_cuda_view = rlt::view<DEVICE_CUDA, typename decltype(matrix_cuda_data)::SPEC, 50, 50>(device_cuda, matrix_cuda_data, 25, 25);

        auto matrix_cpu_view_alt        = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data       )::SPEC, 40, 5>(device_cuda, matrix_cpu_data       ,  5, 5);
        auto matrix_cpu_view_3_alt      = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data_3     )::SPEC, 40, 5>(device_cuda, matrix_cpu_data_3     ,  5, 5);
        auto matrix_cpu_view_3_alt_orig = rlt::view<DEVICE_CUDA, typename decltype(matrix_cpu_data_3_orig)::SPEC, 40, 5>(device_cuda, matrix_cpu_data_3_orig,  5, 5);


        {
            rlt::randn(device_cpu, matrix_cpu_data, rng);
            rlt::randn(device_cpu, matrix_cpu_data_2, rng);
            rlt::randn(device_cpu, matrix_cpu_data_3, rng);
            rlt::copy(device_cpu, device_cpu, matrix_cpu_data_3, matrix_cpu_data_3_orig);
            rlt::copy(device_cpu, device_cuda, matrix_cpu_data_2, matrix_cuda_data);
            rlt::copy(device_cpu, device_cuda, matrix_cpu_view, matrix_cuda_view);
            rlt::copy(device_cuda, device_cpu, matrix_cuda_data, matrix_cpu_data_3);
            DTYPE abs_diff = rlt::abs_diff(device_cpu, matrix_cpu_view, matrix_cpu_view_3);
            EXPECT_LT(abs_diff, 1e-5);
        }
        {
            rlt::randn(device_cpu, matrix_cpu_data, rng);
            rlt::randn(device_cpu, matrix_cpu_data_2, rng);
            rlt::randn(device_cpu, matrix_cpu_data_3, rng);
            rlt::copy(device_cpu, device_cpu, matrix_cpu_data_3, matrix_cpu_data_3_orig);
            rlt::copy(device_cpu, device_cpu, matrix_cpu_view, matrix_cpu_view_3);

            DTYPE abs_diff_3_orig = rlt::abs_diff(device_cpu, matrix_cpu_view_3_alt, matrix_cpu_view_3_alt);
            EXPECT_LT(abs_diff_3_orig, 1e-5);

            rlt::copy(device_cpu, device_cuda, matrix_cpu_data, matrix_cuda_data);
            rlt::copy(device_cpu, device_cuda, matrix_cpu_view_2, matrix_cuda_view);

            rlt::copy(device_cuda, device_cpu, matrix_cuda_data, matrix_cpu_data_4);
            DTYPE abs_diff = rlt::abs_diff(device_cpu, matrix_cpu_data, matrix_cpu_data_4);
            EXPECT_GT(abs_diff, 1e-5);

            rlt::copy(device_cpu, device_cuda, matrix_cpu_view_3, matrix_cuda_view);
            rlt::copy(device_cuda, device_cpu, matrix_cuda_data, matrix_cpu_data_4);
            abs_diff = rlt::abs_diff(device_cpu, matrix_cpu_data, matrix_cpu_data_4);
            EXPECT_LT(abs_diff, 1e-5);
        }



        rlt::free(device_cpu, matrix_cpu_data);
        rlt::free(device_cpu, matrix_cpu_data_2);
        rlt::free(device_cpu, matrix_cpu_data_3);
        rlt::free(device_cpu, matrix_cpu_data_4);
        rlt::free(device_cuda, matrix_cuda_data);
    }
}
template <typename TYPE_POLICY, typename TI>
struct copy{
    static constexpr TI BATCH_SIZE = 100;
    static constexpr TI HIDDEN_DIM = BATCH_SIZE;

    template <rlt::nn::activation_functions::ActivationFunction ACTIVATION_FUNCTION>
    using CONFIGURATION = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, 3, HIDDEN_DIM, ACTIVATION_FUNCTION, ACTIVATION_FUNCTION>;

    using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, OPTIMIZER_PARAMETERS>>;
    template <rlt::nn::activation_functions::ActivationFunction ACTIVATION_FUNCTION>
    using NN = rlt::nn_models::mlp::NeuralNetwork<CONFIGURATION<ACTIVATION_FUNCTION>, rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>, rlt::tensor::Shape<TI, 1, BATCH_SIZE, HIDDEN_DIM>>;

    static constexpr TI ITERATIONS = 1;
    static constexpr TI NAIVE_ITERATIONS = 1;
};


TEST(RL_TOOLS_NN_CUDA, COPY) {
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;
    using T = float;
    using TYPE_POLICY = rlt::numeric_types::Policy<T>;
    using TI_CPU = typename DEVICE_CPU::index_t;
    using TI_CUDA = typename DEVICE_CUDA::index_t;
    using COPY_CPU = copy<TYPE_POLICY, TI_CPU>;
    using COPY_CUDA = copy<TYPE_POLICY, TI_CUDA>;
    using NetworkTypeCPU = COPY_CPU::NN<rlt::nn::activation_functions::RELU>;
    using NetworkTypeCUDA = COPY_CUDA::NN<rlt::nn::activation_functions::RELU>;
    COPY_CPU::OPTIMIZER optimizer_cpu;
    DEVICE_CPU device_cpu;
    DEVICE_CUDA device_cuda;
    NetworkTypeCPU network_cpu;
    NetworkTypeCPU network_cpu_2;
    NetworkTypeCUDA network_cuda;
    rlt::init(device_cuda);
    rlt::malloc(device_cpu, network_cpu);
    rlt::malloc(device_cpu, network_cpu_2);
    rlt::malloc(device_cuda, network_cuda);
    rlt::malloc(device_cpu, optimizer_cpu);

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device_cpu, rng);

    rlt::init_weights(device_cpu, network_cpu, rng);
    rlt::init_weights(device_cpu, network_cpu_2, rng);
    rlt::zero_gradient(device_cpu, network_cpu);
    rlt::zero_gradient(device_cpu, network_cpu_2);
    rlt::init(device_cpu, optimizer_cpu);
    rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu);
    rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu_2);
    rlt::reset_forward_state(device_cpu, network_cpu);
    rlt::reset_forward_state(device_cpu, network_cpu_2);
    auto cpu_network_diff = rlt::abs_diff(device_cpu, network_cpu, network_cpu_2);
    std::cout << "CPU network diff: " << cpu_network_diff << std::endl;
    ASSERT_GT(cpu_network_diff, 0);

    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);
    rlt::copy(device_cuda, device_cpu, network_cuda, network_cpu_2);
    auto cpu_network_diff_round_trip = rlt::abs_diff(device_cpu, network_cpu, network_cpu_2);
    std::cout << "CPU network round-trip: " << cpu_network_diff_round_trip << std::endl;
    ASSERT_FLOAT_EQ(cpu_network_diff_round_trip, 0);

    increment(device_cpu, network_cpu.hidden_layers[0].weights.parameters, 5, 0, 50);

    cpu_network_diff = rlt::abs_diff(device_cpu, network_cpu, network_cpu_2);
    std::cout << "CPU network diff: " << cpu_network_diff << std::endl;
    ASSERT_FLOAT_EQ(cpu_network_diff, 5);

    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);
    rlt::copy(device_cuda, device_cpu, network_cuda, network_cpu_2);
    cpu_network_diff_round_trip = rlt::abs_diff(device_cpu, network_cpu, network_cpu_2);
    ASSERT_FLOAT_EQ(cpu_network_diff_round_trip, 0);
    std::cout << "CPU network round-trip: " << cpu_network_diff_round_trip << std::endl;

    rlt::free(device_cpu, network_cpu);
    rlt::free(device_cpu, network_cpu_2);
    rlt::free(device_cuda, network_cuda);
    rlt::check_status(device_cuda);
}

template <typename TYPE_POLICY, typename TI, TI BATCH_SIZE, TI ITERATIONS>
void GEMM() {
    using T = typename TYPE_POLICY::DEFAULT;
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    constexpr DEVICE_CPU::index_t HIDDEN_DIM = BATCH_SIZE;

    constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::IDENTITY;
    using CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, 3, HIDDEN_DIM, ACTIVATION_FUNCTION, rlt::nn::activation_functions::RELU>;

    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, HIDDEN_DIM>;
    using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, OPTIMIZER_PARAMETERS>>;

    std::cout << "GEMM<" << (rlt::utils::typing::is_same_v<T, float> ? "float" : "double") << ", " << BATCH_SIZE << ">" << std::endl;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using NetworkTypeCPU = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    using NetworkTypeCUDA = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    DEVICE_CPU device_cpu;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    rlt::check_status(device_cuda);
    NetworkTypeCPU network_cpu;
    typename NetworkTypeCPU::template Buffer<> network_cpu_buffers;
    NetworkTypeCUDA network_cuda;
    typename NetworkTypeCUDA::template Buffer<> network_cuda_buffers;
    OPTIMIZER optimizer;
    rlt::malloc(device_cpu, network_cpu);
    rlt::malloc(device_cpu, network_cpu_buffers);
    rlt::check_status(device_cuda);
    rlt::malloc(device_cuda, network_cuda);
    rlt::check_status(device_cuda);
    rlt::malloc(device_cuda, network_cuda_buffers);
    rlt::check_status(device_cuda);
    rlt::malloc(device_cpu, optimizer);
    rlt::check_status(device_cuda);

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device_cpu, rng);
    DEVICE_CUDA::SPEC::RANDOM::ENGINE<> rng_cuda;
    rlt::malloc(device_cuda, rng_cuda);
    rlt::init(device_cuda, rng_cuda);
    rlt::check_status(device_cuda);

    rlt::init_weights(device_cpu, network_cpu, rng);
    rlt::init(device_cpu, optimizer);
    rlt::reset_optimizer_state(device_cpu, optimizer, network_cpu);
    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);

    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, HIDDEN_DIM>> input_cpu;
    rlt::malloc(device_cpu, input_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, NetworkTypeCPU::SPEC::HIDDEN_DIM>> output_first_layer_cpu;
    rlt::malloc(device_cpu, output_first_layer_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, NetworkTypeCPU::SPEC::HIDDEN_DIM>> output_first_layer_cuda_cpu;
    rlt::malloc(device_cpu, output_first_layer_cuda_cpu);

    rlt::randn(device_cpu, input_cpu, rng);


    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, HIDDEN_DIM>> input_cuda;
    rlt::malloc(device_cuda, input_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, NetworkTypeCPU::SPEC::CONFIG::HIDDEN_DIM>> output_first_layer_cuda;
    rlt::malloc(device_cuda, output_first_layer_cuda);

    rlt::copy(device_cpu, device_cuda, input_cpu, input_cuda);

    typename decltype(network_cpu.input_layer)::template Buffer<> buffer_cpu;
    typename decltype(network_cuda.input_layer)::template Buffer<> buffer_cuda;
    rlt::evaluate(device_cpu, network_cpu.input_layer, input_cpu, output_first_layer_cpu, buffer_cpu, rng);
    rlt::evaluate(device_cuda, network_cuda.input_layer, input_cuda, output_first_layer_cuda, buffer_cuda, rng_cuda);
    cudaDeviceSynchronize();

    rlt::copy(device_cuda, device_cpu, output_first_layer_cuda, output_first_layer_cuda_cpu);
    auto evaluation_diff = rlt::abs_diff(device_cpu, output_first_layer_cuda_cpu, output_first_layer_cpu)/(BATCH_SIZE * NetworkTypeCPU::SPEC::CONFIG::HIDDEN_DIM);


    std::cout << "Evaluation diff: " << evaluation_diff << std::endl;
    auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-6 : 1e-15);
    if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
        ASSERT_LT(evaluation_diff, threshold);
    }

    {
        cudaDeviceSynchronize();
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < ITERATIONS; ++i)
        {
            rlt::evaluate(device_cuda, network_cuda.input_layer, input_cuda, output_first_layer_cuda, buffer_cuda, rng_cuda);
            cudaDeviceSynchronize();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "CUDA evaluation time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / ((T)ITERATIONS) << "us" << std::endl;
    }
}
TEST(RL_TOOLS_NN_CUDA, GEMM) {
    using DEFAULT_DTYPE = rlt::numeric_types::Policy<float>;
    GEMM<DEFAULT_DTYPE, unsigned int, 1, 1>();
    GEMM<DEFAULT_DTYPE, unsigned int, 2, 1>();
    GEMM<DEFAULT_DTYPE, unsigned int, 32, 1>();
#ifndef RL_TOOLS_DEBUG
    GEMM<DEFAULT_DTYPE, unsigned int, 1024, 1>();
#endif
    GEMM<DEFAULT_DTYPE, unsigned int, 10, 1>();
    GEMM<DEFAULT_DTYPE, unsigned int, 9, 1>();
    GEMM<rlt::numeric_types::Policy<double>, unsigned int, 200, 1>();
    GEMM<DEFAULT_DTYPE, unsigned int, 200, 1>();
    GEMM<DEFAULT_DTYPE, unsigned int, 64, 1000>();
    GEMM<DEFAULT_DTYPE, unsigned int, 256, 1000>();
}

template <typename TYPE_POLICY, typename TI, TI BATCH_SIZE, TI ITERATIONS>
void FORWARD() {
    using T = typename TYPE_POLICY::DEFAULT;
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    constexpr DEVICE_CPU::index_t HIDDEN_DIM = BATCH_SIZE;

    constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::IDENTITY;
    using CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, HIDDEN_DIM, 3, HIDDEN_DIM, ACTIVATION_FUNCTION, rlt::nn::activation_functions::RELU>;

    using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, typename DEVICE_CUDA::index_t>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;

    std::cout << "FORWARD<" << (rlt::utils::typing::is_same_v<T, float> ? "float" : "double") << ", " << BATCH_SIZE << ">" << std::endl;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, HIDDEN_DIM>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using NetworkTypeCPU = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    using NetworkTypeCUDA = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    DEVICE_CPU device_cpu;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    NetworkTypeCPU network_cpu;
    typename NetworkTypeCPU::template Buffer<> network_cpu_buffers;
    NetworkTypeCUDA network_cuda;
    typename NetworkTypeCPU::template Buffer<> network_cuda_buffers;
    rlt::malloc(device_cpu, network_cpu);
    rlt::malloc(device_cpu, network_cpu_buffers);
    rlt::malloc(device_cuda, network_cuda);
    rlt::malloc(device_cuda, network_cuda_buffers);

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device_cpu, rng);
    DEVICE_CUDA::SPEC::RANDOM::ENGINE<> rng_cuda;
    rlt::malloc(device_cuda, rng_cuda);
    rlt::init(device_cuda, rng_cuda);


    rlt::init_weights(device_cpu, network_cpu, rng);
    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);

    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, rlt::get_last(INPUT_SHAPE{})>> input_cpu;
    rlt::malloc(device_cpu, input_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, rlt::get_last(typename NetworkTypeCPU::OUTPUT_SHAPE{})>> output_cpu;
    rlt::malloc(device_cpu, output_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, rlt::get_last(typename NetworkTypeCPU::OUTPUT_SHAPE{})>> output_cuda_cpu;
    rlt::malloc(device_cpu, output_cuda_cpu);

    rlt::randn(device_cpu, input_cpu, rng);
//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::INPUT_DIM <= 10){
//        std::cout << "Input:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < BATCH_SIZE; ++i)
//        {
//            for(typename NetworkTypeCPU::TI j = 0; j < NetworkTypeCPU::INPUT_DIM; ++j)
//            {
//                std::cout << input_cpu.data[i * NetworkTypeCPU::INPUT_DIM + j] << " ";
//            }
//            std::cout << std::endl;
//        }
//    }
//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::INPUT_DIM <= 10){
//        std::cout << "Weights:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < NetworkTypeCPU::OUTPUT_DIM; ++i)
//        {
//            for(typename NetworkTypeCPU::TI j = 0; j < NetworkTypeCPU::INPUT_DIM; ++j)
//            {
//                std::cout << network_cpu.input_layer.weights.data[i * NetworkTypeCPU::INPUT_DIM + j] << " ";
//            }
//            std::cout << std::endl;
//        }
//    }
//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::INPUT_DIM <= 10){
//        std::cout << "Biases:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < NetworkTypeCPU::OUTPUT_DIM; ++i)
//        {
//            std::cout << network_cpu.input_layer.biases.data[i] << " ";
//        }
//        std::cout << std::endl;
//    }


    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, rlt::get_last(INPUT_SHAPE{})>> input_cuda;
    rlt::malloc(device_cuda, input_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, rlt::get_last(typename NetworkTypeCPU::OUTPUT_SHAPE{})>> output_cuda;
    rlt::malloc(device_cuda, output_cuda);

    rlt::copy(device_cpu, device_cuda, input_cpu, input_cuda);

    rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
    rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng_cuda);
    cudaDeviceSynchronize();

    rlt::copy(device_cuda, device_cpu, network_cuda.output_layer.output, output_cuda_cpu);
    auto evaluation_diff = rlt::abs_diff(device_cpu, output_cuda_cpu, network_cpu.output_layer.output)/(BATCH_SIZE * rlt::get_last(typename NetworkTypeCPU::OUTPUT_SHAPE{}));

//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::OUTPUT_DIM <= 10){
//        std::cout << "cpu output:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < BATCH_SIZE; ++i)
//        {
//            for(typename NetworkTypeCPU::TI j = 0; j < NetworkTypeCPU::OUTPUT_DIM; ++j)
//            {
//                std::cout << output_cpu.data[i * NetworkTypeCPU::OUTPUT_DIM + j] << " ";
//            }
//            std::cout << std::endl;
//        }
//    }
//
//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::OUTPUT_DIM <= 10){
//        std::cout << "cuda output:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < BATCH_SIZE; ++i){
//            for(typename NetworkTypeCPU::TI j = 0; j < NetworkTypeCPU::OUTPUT_DIM; ++j){
//                std::cout << output_cuda_cpu.data[i * NetworkTypeCPU::OUTPUT_DIM + j] << " ";
//            }
//            std::cout << std::endl;
//        }
//    }
//
//    if(BATCH_SIZE <= 10 && NetworkTypeCPU::OUTPUT_DIM <= 10){
//        std::cout << "cuda diff:" << std::endl;
//        for(typename NetworkTypeCPU::TI i = 0; i < BATCH_SIZE; ++i)
//        {
//            for(typename NetworkTypeCPU::TI j = 0; j < NetworkTypeCPU::OUTPUT_DIM; ++j)
//            {
//                T diff = output_cpu.data[i * NetworkTypeCPU::OUTPUT_DIM + j] - output_cuda_cpu.data[i * NetworkTypeCPU::OUTPUT_DIM + j];
//                diff = std::abs(diff) > 1e-7 ? diff : 0;
//                std::cout << diff << " ";
//            }
//            std::cout << std::endl;
//        }
//    }

    std::cout << "Evaluation diff: " << evaluation_diff << std::endl;
    auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-7 : 1e-15);
    if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
        ASSERT_LT(evaluation_diff, threshold);
    }

    {
        cudaDeviceSynchronize();
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < ITERATIONS; ++i)
        {
            rlt::evaluate(device_cuda, network_cuda, input_cuda, output_cuda, network_cuda_buffers, rng_cuda);
            cudaDeviceSynchronize();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "CUDA evaluation time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / ((T)ITERATIONS) << "us" << std::endl;
    }
}

TEST(RL_TOOLS_NN_CUDA, FORWARD) {
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 1, 1>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 2, 1>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 32, 1>();
#ifndef RL_TOOLS_DEBUG
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 1024, 1>();
#endif
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 10, 1>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 9, 1>();
    FORWARD<rlt::numeric_types::Policy<double>, unsigned int, 200, 1>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 200, 1>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 64, 1000>();
    FORWARD<rlt::numeric_types::Policy<float>, unsigned int, 256, 100>();
}

template <typename TYPE_POLICY, typename TI, TI BATCH_SIZE, TI INPUT_DIM, TI HIDDEN_DIM, TI OUTPUT_DIM, TI ITERATIONS>
void BACKWARD() {
    using T = typename TYPE_POLICY::DEFAULT;
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::IDENTITY;
    using CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, 3, HIDDEN_DIM, rlt::nn::activation_functions::RELU, ACTIVATION_FUNCTION>;

    using OPTIMIZER_PARAMETERS = rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_PYTORCH<TYPE_POLICY>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, OPTIMIZER_PARAMETERS>>;

    std::cout << "BACKWARD<" << (rlt::utils::typing::is_same_v<T, float> ? "float" : "double") << ", " << BATCH_SIZE << ">" << std::endl;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using NetworkTypeCPU = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    using NetworkTypeCUDA = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    DEVICE_CPU device_cpu;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    NetworkTypeCPU network_cpu;
    NetworkTypeCPU network_cpu_pre;
    NetworkTypeCPU network_cuda_cpu;
    typename NetworkTypeCPU::template Buffer<> network_cpu_buffers;
    NetworkTypeCUDA network_cuda;
    typename NetworkTypeCPU::template Buffer<> network_cuda_buffers;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> d_output_cpu;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> d_output_cuda;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, INPUT_DIM>> d_input_cpu;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, INPUT_DIM>> d_input_cuda;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, INPUT_DIM>> d_input_cuda_cpu;
    rlt::malloc(device_cuda, d_output_cuda);
    rlt::malloc(device_cpu, d_output_cpu);
    rlt::malloc(device_cuda, d_input_cuda);
    rlt::malloc(device_cpu, d_input_cpu);
    rlt::malloc(device_cpu, d_input_cuda_cpu);
    OPTIMIZER optimizer_cpu, optimizer_cuda;
    rlt::malloc(device_cpu, network_cpu);
    rlt::malloc(device_cpu, network_cpu_pre);
    rlt::malloc(device_cpu, network_cuda_cpu);
    rlt::malloc(device_cpu, network_cpu_buffers);
    rlt::malloc(device_cuda, network_cuda);
    rlt::malloc(device_cuda, network_cuda_buffers);
    rlt::malloc(device_cpu, optimizer_cpu);
    rlt::malloc(device_cuda, optimizer_cuda);

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device_cpu, rng);
    DEVICE_CUDA::SPEC::RANDOM::ENGINE<> rng_cuda;
    rlt::malloc(device_cuda, rng_cuda);
    rlt::init(device_cuda, rng_cuda);

    rlt::init_weights(device_cpu, network_cpu, rng);
    rlt::zero_gradient(device_cpu, network_cpu);
    rlt::init(device_cpu, optimizer_cpu);
    rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu);
    rlt::copy(device_cpu, device_cpu, network_cpu, network_cpu_pre);

    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, INPUT_DIM>> input_cpu;
    rlt::malloc(device_cpu, input_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cpu;
    rlt::malloc(device_cpu, output_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_target_cpu;
    rlt::malloc(device_cpu, output_target_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cuda_cpu;
    rlt::malloc(device_cpu, output_cuda_cpu);

//    for(typename NetworkTypeCPU::TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
//        for(typename NetworkTypeCPU::TI input_i = 0; input_i < NetworkTypeCPU::INPUT_DIM; input_i++){
//            set(input_cpu, batch_i, input_i, rlt::random::normal_distribution(DEVICE_CPU::SPEC::RANDOM(), (T)0, (T)1, rng));
//        }
//    }
//    for(typename NetworkTypeCPU::TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
//        for(typename NetworkTypeCPU::TI input_i = 0; input_i < NetworkTypeCPU::OUTPUT_DIM; input_i++){
//            set(output_target_cpu, batch_i, input_i, rlt::random::normal_distribution(DEVICE_CPU::SPEC::RANDOM(), (T)0, (T)1, rng));
//        }
//    }
    rlt::randn(device_cpu, input_cpu, rng);
    rlt::randn(device_cpu, output_target_cpu, rng);

    rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
    rlt::nn::loss_functions::mse::gradient(device_cpu, rlt::output(network_cpu), output_target_cpu, d_output_cpu);
    rlt::backward(device_cpu, network_cpu, input_cpu, d_output_cpu, network_cpu_buffers);
    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);


    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, INPUT_DIM>> input_cuda;
    rlt::malloc(device_cuda, input_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cuda;
    rlt::malloc(device_cuda, output_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_target_cuda;
    rlt::malloc(device_cuda, output_target_cuda);

    rlt::copy(device_cpu, device_cuda, input_cpu, input_cuda);
    rlt::copy(device_cpu, device_cuda, output_target_cpu, output_target_cuda);

    rlt::zero_gradient(device_cpu, network_cpu);
    rlt::zero_gradient(device_cuda, network_cuda);
//    rlt::forward_backward_mse(device_cpu, network_cpu, input_cpu, output_target_cpu, network_cpu_buffers);
    {
        rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
        rlt::nn::loss_functions::mse::gradient(device_cpu, rlt::output(network_cpu), output_target_cpu, d_output_cpu);
        rlt::backward(device_cpu, network_cpu, input_cpu, d_output_cpu, network_cpu_buffers);
    }
//    rlt::forward_backward_mse(device_cuda, network_cuda, input_cuda, output_target_cuda, network_cuda_buffers);
    {
        rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng_cuda);
        rlt::nn::loss_functions::mse::gradient(device_cuda, rlt::output(network_cuda), output_target_cuda, d_output_cuda);
        rlt::backward(device_cuda, network_cuda, input_cuda, d_output_cuda, network_cuda_buffers);
    }
    cudaDeviceSynchronize();

    rlt::copy(device_cuda, device_cpu, network_cuda, network_cuda_cpu);
//    auto evaluation_diff_pre = rlt::abs_diff(device_cpu, network_cuda_cpu, network_cpu_pre)/(BATCH_SIZE * NetworkTypeCPU::OUTPUT_DIM);
    auto evaluation_diff = rlt::abs_diff(device_cpu, network_cuda_cpu, network_cpu)/(BATCH_SIZE * OUTPUT_DIM);


    std::cout << "Evaluation diff: " << evaluation_diff << std::endl;
    auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-6 : 1e-14);
    if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
        ASSERT_LT(evaluation_diff, threshold);
    }
    {

        rlt::init(device_cpu, optimizer_cpu);
        rlt::init(device_cuda, optimizer_cuda);
        rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu);
        rlt::reset_optimizer_state(device_cuda, optimizer_cuda, network_cuda);
        rlt::zero_gradient(device_cpu, network_cpu);
        rlt::zero_gradient(device_cuda, network_cuda);
        {
            rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
            rlt::nn::loss_functions::mse::gradient(device_cpu, rlt::output(network_cpu), output_target_cpu, d_output_cpu);
            rlt::backward_input(device_cpu, network_cpu, d_output_cpu, d_input_cpu, network_cpu_buffers);
        }
        {
            rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng_cuda);
            rlt::nn::loss_functions::mse::gradient(device_cuda, rlt::output(network_cuda), output_target_cuda, d_output_cuda);
            rlt::backward_input(device_cuda, network_cuda, d_output_cuda, d_input_cuda, network_cuda_buffers);
        }
        cudaDeviceSynchronize();

        {
            rlt::copy(device_cuda, device_cpu, network_cuda, network_cuda_cpu);
            auto evaluation_diff = rlt::abs_diff(device_cpu, network_cuda_cpu, network_cpu)/(BATCH_SIZE * OUTPUT_DIM);


            std::cout << "Evaluation diff: " << evaluation_diff << std::endl;
            auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-6 : 1e-14);
            if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
                ASSERT_LT(evaluation_diff, threshold);
            }
        }
        {

            rlt::copy(device_cuda, device_cpu, d_input_cuda, d_input_cuda_cpu);
            auto evaluation_diff = rlt::abs_diff(device_cpu, d_input_cuda_cpu, d_input_cpu);


            std::cout << "d_input diff: " << evaluation_diff << std::endl;
            auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-6 : 1e-14);
            if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
                ASSERT_LT(evaluation_diff, threshold);
            }
        }
    }

    {
        cudaDeviceSynchronize();
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < ITERATIONS; ++i)
        {
//            rlt::forward_backward_mse(device_cuda, network_cuda, input_cuda, output_target_cuda, network_cuda_buffers);
            {
                rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng);
                rlt::nn::loss_functions::mse::gradient(device_cuda, rlt::output(network_cuda), output_target_cuda, d_output_cuda);
                rlt::backward(device_cuda, network_cuda, input_cuda, d_output_cuda, network_cuda_buffers);
            }
            cudaDeviceSynchronize();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "CUDA evaluation time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / ((T)ITERATIONS) << "us" << std::endl;
    }
}

TEST(RL_TOOLS_NN_CUDA, BACKWARD) {
    using DEFAULT_DTYPE = rlt::numeric_types::Policy<float>;
    BACKWARD<DEFAULT_DTYPE, unsigned int,    1, 1, 1, 1, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,    1, 256,  10, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,    2, 256,  10, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,   32, 256,  10, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int, 1024, 256,  10, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,   10, 256, 200, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,    9, 256,  60, 100, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,  200, 256,  11, 100, 1>();
    BACKWARD<rlt::numeric_types::Policy<double>, unsigned int,  200, 256,  12, 101, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,   64, 256,  50, 101, 1>();
    BACKWARD<DEFAULT_DTYPE, unsigned int,  256, 256, 256, 256, 100>();
}

template <typename TYPE_POLICY, typename TI, TI BATCH_SIZE, TI INPUT_DIM, TI HIDDEN_DIM, TI OUTPUT_DIM, TI ITERATIONS>
void ADAM_UPDATE() {
    using T = typename TYPE_POLICY::DEFAULT;
    using DEVICE_CPU = rlt::devices::DefaultCPU;
    using DEVICE_CUDA = rlt::devices::DefaultCUDA;

    constexpr auto ACTIVATION_FUNCTION = rlt::nn::activation_functions::IDENTITY;
    using CONFIG = rlt::nn_models::mlp::Configuration<TYPE_POLICY, TI, OUTPUT_DIM, 3, HIDDEN_DIM, rlt::nn::activation_functions::RELU, ACTIVATION_FUNCTION>;

    using OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI>;
    using OPTIMIZER = rlt::nn::optimizers::Adam<OPTIMIZER_SPEC>;

    std::cout << "BACKWARD<" << (rlt::utils::typing::is_same_v<T, float> ? "float" : "double") << ", " << BATCH_SIZE << ">" << std::endl;
    using INPUT_SHAPE = rlt::tensor::Shape<TI, 1, BATCH_SIZE, INPUT_DIM>;
    using CAPABILITY = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
    using NetworkTypeCPU = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    using NetworkTypeCUDA = rlt::nn_models::mlp::NeuralNetwork<CONFIG, CAPABILITY, INPUT_SHAPE>;
    DEVICE_CPU device_cpu;
    DEVICE_CUDA device_cuda;
    rlt::init(device_cuda);
    NetworkTypeCPU network_cpu;
    NetworkTypeCPU network_cpu_pre;
    NetworkTypeCPU network_cuda_cpu;
    typename NetworkTypeCPU::template Buffer<> network_cpu_buffers;
    NetworkTypeCUDA network_cuda;
    typename NetworkTypeCPU::template Buffer<> network_cuda_buffers;
    OPTIMIZER optimizer_cpu, optimizer_cuda;
    rlt::malloc(device_cpu, network_cpu);
    rlt::malloc(device_cpu, network_cpu_pre);
    rlt::malloc(device_cpu, network_cuda_cpu);
    rlt::malloc(device_cpu, network_cpu_buffers);
    rlt::malloc(device_cuda, network_cuda);
    rlt::malloc(device_cuda, network_cuda_buffers);
    rlt::malloc(device_cpu, optimizer_cpu);
    rlt::malloc(device_cuda, optimizer_cuda);

    DEVICE_CPU::SPEC::RANDOM::ENGINE<> rng;
    rlt::init(device_cpu, rng);
    DEVICE_CUDA::SPEC::RANDOM::ENGINE<> rng_cuda;
    rlt::malloc(device_cuda, rng_cuda);
    rlt::init(device_cuda, rng_cuda);
    rlt::init(device_cpu, optimizer_cpu);
    rlt::init(device_cuda, optimizer_cuda);

    rlt::init_weights(device_cpu, network_cpu, rng);
    rlt::zero_gradient(device_cpu, network_cpu);
    rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu);
    rlt::reset_optimizer_state(device_cuda, optimizer_cuda, network_cuda);
    rlt::copy(device_cpu, device_cpu, network_cpu, network_cpu_pre);

    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, INPUT_DIM>> input_cpu;
    rlt::malloc(device_cpu, input_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cpu;
    rlt::malloc(device_cpu, output_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_target_cpu;
    rlt::malloc(device_cpu, output_target_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cuda_cpu;
    rlt::malloc(device_cpu, output_cuda_cpu);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> d_output_cpu;
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> d_output_cuda;
    rlt::malloc(device_cuda, d_output_cuda);
    rlt::malloc(device_cpu, d_output_cpu);

    rlt::randn(device_cpu, input_cpu, rng);
    rlt::randn(device_cpu, output_target_cpu, rng);
    {
        rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
        rlt::nn::loss_functions::mse::gradient(device_cpu, rlt::output(network_cpu), output_target_cpu, d_output_cpu);
        rlt::backward(device_cpu, network_cpu, input_cpu, d_output_cpu, network_cpu_buffers);
    }
    rlt::copy(device_cpu, device_cuda, network_cpu, network_cuda);


    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, INPUT_DIM>> input_cuda;
    rlt::malloc(device_cuda, input_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CUDA::index_t, BATCH_SIZE, OUTPUT_DIM>> output_cuda;
    rlt::malloc(device_cuda, output_cuda);
    rlt::Matrix<rlt::matrix::Specification<T, DEVICE_CPU::index_t, BATCH_SIZE, OUTPUT_DIM>> output_target_cuda;
    rlt::malloc(device_cuda, output_target_cuda);

    rlt::copy(device_cpu, device_cuda, input_cpu, input_cuda);
    rlt::copy(device_cpu, device_cuda, output_target_cpu, output_target_cuda);

    rlt::zero_gradient(device_cpu, network_cpu);
    rlt::zero_gradient(device_cuda, network_cuda);
    rlt::reset_optimizer_state(device_cpu, optimizer_cpu, network_cpu);
    rlt::reset_optimizer_state(device_cuda, optimizer_cuda, network_cuda);
//    rlt::forward_backward_mse(device_cpu, network_cpu, input_cpu, output_target_cpu, network_cpu_buffers);
    {
        rlt::forward(device_cpu, network_cpu, input_cpu, network_cpu_buffers, rng);
        rlt::nn::loss_functions::mse::gradient(device_cpu, rlt::output(network_cpu), output_target_cpu, d_output_cpu);
        rlt::backward(device_cpu, network_cpu, input_cpu, d_output_cpu, network_cpu_buffers);
    }
    rlt::step(device_cpu, optimizer_cpu, network_cpu);
//    rlt::forward_backward_mse(device_cuda, network_cuda, input_cuda, output_target_cuda, network_cuda_buffers);
    {
        rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng_cuda);
        rlt::nn::loss_functions::mse::gradient(device_cuda, rlt::output(network_cuda), output_target_cuda, d_output_cuda);
        rlt::backward(device_cuda, network_cuda, input_cuda, d_output_cuda, network_cuda_buffers);
    }
    rlt::step(device_cuda, optimizer_cuda, network_cuda);
    cudaDeviceSynchronize();

    rlt::copy(device_cuda, device_cpu, network_cuda, network_cuda_cpu);
    auto evaluation_diff_pre = rlt::abs_diff(device_cpu, network_cuda_cpu, network_cpu_pre)/(BATCH_SIZE * OUTPUT_DIM);
    auto evaluation_diff = rlt::abs_diff(device_cpu, network_cuda_cpu, network_cpu)/(BATCH_SIZE * OUTPUT_DIM);

    std::cout << "Evaluation diff: " << evaluation_diff << std::endl;
    auto threshold = (rlt::utils::typing::is_same_v<T, float> ? 1e-6 : 1e-14);
    if(std::isnan(evaluation_diff) || evaluation_diff > threshold){
        ASSERT_LT(evaluation_diff, threshold);
    }

    {
        cudaDeviceSynchronize();
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < ITERATIONS; ++i)
        {
//            rlt::forward_backward_mse(device_cuda, network_cuda, input_cuda, output_target_cuda, network_cuda_buffers);
            {
                rlt::forward(device_cuda, network_cuda, input_cuda, network_cuda_buffers, rng_cuda);
                rlt::nn::loss_functions::mse::gradient(device_cuda, rlt::output(network_cuda), output_target_cuda, d_output_cuda);
                rlt::backward(device_cuda, network_cuda, input_cuda, d_output_cuda, network_cuda_buffers);
            }
            rlt::step(device_cuda, optimizer_cuda, network_cuda);
            cudaDeviceSynchronize();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "CUDA evaluation time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / ((T)ITERATIONS) << "us" << std::endl;
    }
}

TEST(RL_TOOLS_NN_CUDA, ADAM_UPDATE) {
    using DEFAULT_DTYPE = rlt::numeric_types::Policy<float>;
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,    1, 256,  10, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,    2, 256,  10, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,   32, 256,  10, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int, 1024, 256,  10, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,   10, 256, 200, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,    9, 256,  60, 100, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,  200, 256,  11, 100, 1>();
    ADAM_UPDATE<rlt::numeric_types::Policy<double>, unsigned int,  200, 256,  12, 101, 1>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,   64, 256,  50, 101, 100>();
    ADAM_UPDATE<DEFAULT_DTYPE, unsigned int,  256, 256, 256, 256, 100>();
}