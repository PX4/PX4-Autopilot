#if defined(RL_TOOLS_BACKEND_ENABLE_MKL) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../../../rl/algorithms/sac/operations_cpu_mkl.h"
#else
#if defined(RL_TOOLS_BACKEND_ENABLE_ACCELERATE) && !defined(RL_TOOLS_BACKEND_DISABLE_BLAS)
#include "../../../rl/algorithms/sac/operations_cpu_accelerate.h"
#else
#include "../../../rl/algorithms/sac/operations_cpu.h"
#endif
#endif
