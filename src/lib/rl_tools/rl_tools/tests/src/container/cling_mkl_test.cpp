#!/usr/bin/env cling -std=c++17
#pragma cling add_include_path("/home/jonas/phd/projects/rl_for_control/rl_tools/include")
#pragma cling add_include_path("/opt/intel/oneapi/mkl/latest/include")
#pragma cling add_library_path("/opt/intel/oneapi/mkl/latest/lib/intel64/")
#pragma cling add_library_path("/opt/intel/oneapi/compiler/latest/linux/compiler/lib/intel64_lin")

#define RL_TOOLS_BACKEND_ENABLE_MKL
#include <rl_tools/operations/cpu_mux.h>

#ifdef RL_TOOLS_BACKEND_ENABLE_MKL
#pragma cling load("mkl_core")
#pragma cling load("mkl_intel_thread")
#pragma cling load("mkl_intel_ilp64")
#pragma cling load("iomp5")
#pragma cling load("lirlthread.so.0")
#pragma cling load("libm.so.6")
#pragma cling load("libdl.so.2")
#endif

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using DEVICE_MKL = rlt::devices::DefaultCPU_MKL;
using T = float;
using TI = typename DEVICE::index_t;

rlt::Matrix<rlt::matrix::Specification<T, TI, 2, 2>> A, B, C, C_mkl;
DEVICE device;
DEVICE_MKL device_mkl;
DEVICE::SPEC::RANDOM::ENGINE<> rng;
rlt::malloc(device, rng);
rlt::init(device, rng, 1);

rlt::malloc(device, A);
rlt::malloc(device, B);
rlt::malloc(device, C);
rlt::malloc(device, C_mkl);
rlt::randn(device, A, rng);
rlt::randn(device, B, rng);

rlt::print_python_literal(device, A);
rlt::print_python_literal(device, B);

rlt::multiply(device, A, B, C);
rlt::print_python_literal(device, C);

rlt::multiply(device_mkl, A, B, C_mkl);
rlt::print_python_literal(device, C_mkl);

auto diff = rlt::abs_diff(device, C, C_mkl);
std::cout << "diff: " << diff << std::endl;