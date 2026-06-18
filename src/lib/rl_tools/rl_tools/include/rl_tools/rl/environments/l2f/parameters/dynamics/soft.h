#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_SOFT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_SOFT_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    template<typename T, typename TI>
    constexpr Dynamics<T, TI, 4> soft = {
            // Rotor positions
            {
                    {
                      0.142,
                      -0.169,
                      0
                    },
                    {
                      -0.142,
                       0.169,
                      0
                    },
                    {
                      0.142,
                      0.169,
                      0
                    },
                    {
                      -0.142,
                      -0.169,
                      0
                    }
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
                    {-0.13662779341843762, 4.247460030890121, 3.766033614431835},
                    {-0.13662779341843762, 4.247460030890121, 3.766033614431835},
                    {-0.13662779341843762, 4.247460030890121, 3.766033614431835},
                    {-0.13662779341843762, 4.247460030890121, 3.766033614431835},
            },
            // torque constant
            {0.2, 0.2, 0.2, 0.2},
            // T, RPM time constant
            {
                    0.040,
                    0.040,
                    0.040,
                    0.040
            },
            {
                    0.040,
                    0.040,
                    0.040,
                    0.040
            },
            // mass vehicle
            0.970,
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            8.1547e-3,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            5.2261e-3,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            12.0427e-3
                    }
            },
            // J_inv
            {
                    {
                            122.62866813003542,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            191.3472761715237,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            83.0378569589876
                    }
            },
            0.7261389721508553, // "hovering_throttle_relative"
            // action limit
            {0, 1},
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif