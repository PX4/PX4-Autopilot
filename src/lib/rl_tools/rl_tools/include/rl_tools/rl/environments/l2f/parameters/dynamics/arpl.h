#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_ARPL_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_ARPL_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    template<typename T, typename TI>
    constexpr Dynamics<T, TI, 4> arpl = {
            // Rotor positions
            {
                    {
                            0.0775,
                            -0.0981,
                            0

                    },
                    {
                            -0.0775,
                            0.0981,
                            0

                    },
                    {
                            0.0775,
                            0.0981,
                            0

                    },
                    {
                            -0.0775,
                            -0.0981,
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
                    {-0.5300192490953086, 8.971435376396558, -0.8911788522090669},
                    {-0.5300192490953086, 8.971435376396558, -0.8911788522090669},
                    {-0.5300192490953086, 8.971435376396558, -0.8911788522090669},
                    {-0.5300192490953086, 8.971435376396558, -0.8911788522090669}
            },
            // torque constant
            {0.005964552, 0.005964552, 0.005964552, 0.005964552},
            // T, RPM time constant
            {0.033, 0.033, 0.033, 0.033},
            {0.033, 0.033, 0.033, 0.033},
            // mass vehicle
            0.782,
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            1.7665e-3,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            1.3766e-3,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            (1.7665e-3 + 1.3766e-3)/2 * 1.879
                    }
            },
            // J_inv
            {
                    {
                            1/1.7665e-3,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            1/1.3766e-3,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            1/((1.7665e-3 + 1.3766e-3)/2 * 1.879)
                    }
            },
            // hovering throttle (julia): sqrt((mass * 9.81/4 - thrust_curve[1])/thrust_curve[3]),
            0.13095934350152208,
            // action limit
            {0.0, 1.0},
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif