#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../nn/operations_cpu_mkl.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../nn/operations_cpu_accelerate.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_OPENBLAS) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../nn/operations_cpu_openblas.h"
#else
#include "../nn/operations_generic.h"
#endif
#endif
#endif
#if defined(RL_TOOLS_BACKEND_ENABLE_CUDA) && defined(RL_TOOLS_OPERATIONS_CPU_MUX_INCLUDE_CUDA)
#include "../nn/operations_cuda.h"
#endif
