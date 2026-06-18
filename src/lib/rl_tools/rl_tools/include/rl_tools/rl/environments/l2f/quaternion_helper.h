#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_QUATERNION_HELPER_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_QUATERNION_HELPER_H

#include <rl_tools/utils/generic/vector_operations.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f{
    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void quaternion_derivative(const T q[4], const T omega[3], T q_dot[4]) {
        // FLOPS: 3 (MAC) * 4 + 4 = 16
        q_dot[0] = -q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2];
        q_dot[1] =  q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1];
        q_dot[2] =  q[0]*omega[1] + q[3]*omega[0] - q[1]*omega[2];
        q_dot[3] =  q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0];
        rl_tools::utils::vector_operations::scalar_multiply<DEVICE, T, 4>(q_dot, 0.5);
    }


    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void rotate_vector_by_quaternion(const T q[4], const T v[3], T v_out[3]) {
        using namespace rl_tools::utils::vector_operations;
//    v_out[0] = q[0]*(q[0]*v[0] + q[2]*v[2] - q[3]*v[1]) + q[2]*(q[1]*v[1] + q[0]*v[2] - q[2]*v[0]) - q[3]*(q[0]*v[1] + q[3]*v[0] - q[1]*v[2]) - q[1]*(-q[1]*v[0] - q[2]*v[1] - q[3]*v[2]);
//    v_out[1] = q[0]*(q[0]*v[1] + q[3]*v[0] - q[1]*v[2]) + q[3]*(q[0]*v[0] + q[2]*v[2] - q[3]*v[1]) - q[1]*(q[1]*v[1] + q[0]*v[2] - q[2]*v[0]) - q[2]*(-q[1]*v[0] - q[2]*v[1] - q[3]*v[2]);
//    v_out[2] = q[0]*(q[1]*v[1] + q[0]*v[2] - q[2]*v[0]) + q[1]*(q[0]*v[1] + q[3]*v[0] - q[1]*v[2]) - q[2]*(q[0]*v[0] + q[2]*v[2] - q[3]*v[1]) - q[3]*(-q[1]*v[0] - q[2]*v[1] - q[3]*v[2]);
        // FLOPS: 6 + 3 + 6 + 3 + 3 = 21

        T var[3];
        cross_product<DEVICE, T>(&q[1], v, var); // 6 flops
        scalar_multiply<DEVICE, T, 3>(var, 2); // 3 flops
        cross_product<DEVICE, T>(&q[1], var, v_out); // 6 flops
        rl_tools::utils::vector_operations::scalar_multiply_accumulate<DEVICE, T, 3>(var, q[0], v_out); // 3 flops
        add_accumulate<DEVICE, T, 3>(v, v_out); // 3 flops
    }

    // quaternion to rotation matrix
    template <typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT void quaternion_to_rotation_matrix(const T q[4], T R[3][3]) {
        // w = q[0], x = q[1], y = q[2], z = q[3]
        R[0][0] = 1 - 2*q[2]*q[2] - 2*q[3]*q[3];
        R[0][1] = 2*q[1]*q[2] - 2*q[0]*q[3];
        R[0][2] = 2*q[1]*q[3] + 2*q[0]*q[2];
        R[1][0] = 2*q[1]*q[2] + 2*q[0]*q[3];
        R[1][1] = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
        R[1][2] = 2*q[2]*q[3] - 2*q[0]*q[1];
        R[2][0] = 2*q[1]*q[3] - 2*q[0]*q[2];
        R[2][1] = 2*q[2]*q[3] + 2*q[0]*q[1];
        R[2][2] = 1 - 2*q[1]*q[1] - 2*q[2]*q[2];
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END


#endif