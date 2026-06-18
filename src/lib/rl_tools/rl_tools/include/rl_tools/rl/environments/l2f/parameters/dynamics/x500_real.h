#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_X500_REAL_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_X500_REAL_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    namespace x500{
        template<typename T, typename TI>
        constexpr Dynamics<T, TI, 4> real = {
            // Rotor positions
            {
                {
                    +0.176776695296636,
                    -0.176776695296636,
                    0
                },
                {
                    -0.176776695296636,
                    +0.176776695296636,
                    0
                },
                {
                    +0.176776695296636,
                    +0.176776695296636,
                    0
                },
                {
                    -0.176776695296636,
                    -0.176776695296636,
                    0
                },
            },
            // Rotor thrust directions
            {
                {0, 0, 1},
                {0, 0, 1},
                {0, 0, 1},
                {0, 0, 1},
            },
            // Rotor torque directions
            {
                {0, 0, -1},
                {0, 0, -1},
                {0, 0, +1},
                {0, 0, +1},
            },
            // thrust constants
            {
                {0, 0, 15.926119900619797},
                {0, 0, 15.926119900619797},
                {0, 0, 15.926119900619797},
                {0, 0, 15.926119900619797}
            },
            // torque constant
            {1.0847e-1, 1.0847e-1, 1.0847e-1, 1.0847e-1},
            // T, RPM time constant
            {0.054, 0.054, 0.054, 0.054},
            {0.054, 0.054, 0.054, 0.054},
            // mass vehicle
            2.000,
            // gravity
            {0, 0, -9.81},
            // J
            {
                {2.1705-2, 0.0, 0.0},
                {0.0, 2.1304e-2, 0.0},
                {0.0, 0.0, 0.039396244}
            },
            // J_inv
            {
                {46.072333563695004, 0.0, 0.0},
                {0.0, 46.939541870071345, 0.0},
                {0.0, 0.0, 25.38313043243412}
            },
            // hovering throttle (julia):
            // f(coeffs, q) = [(-coeffs[2] + s * sqrt(coeffs[2]^2 - 4*coeffs[3]*(coeffs[1] - q))) / (2*coeffs[3]) for s in (1, -1)]
            // f([0, 0, 15.926119900619797], 9.81 * 2.0 / 4) = 0.5549636212401485
            0.5549636212401485,
            // action limit
            {0.0, 1.0},
        };
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif