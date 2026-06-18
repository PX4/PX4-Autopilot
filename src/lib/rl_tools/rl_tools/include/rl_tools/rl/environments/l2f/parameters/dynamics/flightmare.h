#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_FLIGHTMARE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_FLIGHTMARE_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    template<typename T, typename TI>
    constexpr Dynamics<T, TI, 4> flightmare = {
            // Rotor positions
            {
                    {
                            0.17 * 0.7071067811865475,
                            -0.17 * 0.7071067811865475,
                            0

                    },
                    {
                            -0.17 * 0.7071067811865475,
                            0.17 * 0.7071067811865475,
                            0

                    },
                    {
                            0.17 * 0.7071067811865475,
                            0.17 * 0.7071067811865475,
                            0

                    },
                    {
                            -0.17 * 0.7071067811865475,
                            -0.17 * 0.7071067811865475,
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
                    {-1.7689986848125325, 11.50824315802381, 11.968428150335603}, // orig: [1.3298253500372892e-06, 0.0038360810526746033, -1.7689986848125325] (x^2, x, 1) => (1, x, x^2) => (1, 3000, 3000^2) (to normalize to 0-1)
                    {-1.7689986848125325, 11.50824315802381, 11.968428150335603},
                    {-1.7689986848125325, 11.50824315802381, 11.968428150335603},
                    {-1.7689986848125325, 11.50824315802381, 11.968428150335603}
            },
            // torque constant
            {0.016, 0.016, 0.016, 0.016},
            // T, RPM time constant
            {0.0200, 0.0200, 0.0200, 0.0200},
            {0.0200, 0.0200, 0.0200, 0.0200},
            // mass vehicle
            0.73,
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            0.00791,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            0.00791,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            0.0123
                    }
            },
            // J_inv
            {
                    {
                            126.42225031605562,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            126.42225031605562,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            81.30081300813008
                    }
            },
            0.5,
            // action limit
            {0, 1.0},
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif