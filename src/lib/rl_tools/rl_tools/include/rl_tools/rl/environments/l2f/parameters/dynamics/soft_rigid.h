#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_SOFT_RIGID_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_SOFT_RIGID_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    template<typename T, typename TI>
    constexpr Dynamics<T, TI, 4> soft_rigid = {
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

                    {-0.33379755540556755, 6.090995069311198, 0.7818335456716792},
                    {-0.33379755540556755, 6.090995069311198, 0.7818335456716792},
                    {-0.33379755540556755, 6.090995069311198, 0.7818335456716792},
                    {-0.33379755540556755, 6.090995069311198, 0.7818335456716792}
            },
            // torque constant
            {1.7602e-2, 1.7602e-2, 1.7602e-2, 1.7602e-2},
            // T, RPM time constant
            {
                    0.046,
                    0.046,
                    0.046,
                    0.046
            },
            {
                    0.046,
                    0.046,
                    0.046,
                    0.046
            },
            // mass vehicle
            0.804,
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            6.5842e-3,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            4.9571e-3,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            0.010571830800000001
                    }
            },
            // J_inv
            {
                    {
                            151.87874001397284,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            201.73085069899741,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            94.59099553504015
                    }
            },
            0.7261389721508553, // "hovering_throttle_relative"
            // action limit
            {0, 1},
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif