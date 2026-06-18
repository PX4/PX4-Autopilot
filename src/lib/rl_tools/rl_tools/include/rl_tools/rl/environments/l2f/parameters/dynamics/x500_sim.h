#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_X500_SIM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_X500_SIM_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    namespace x500{
        template<typename T, typename TI>
        constexpr Dynamics<T, TI, 4> sim = {
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
                {0, 0, 8.74},
                {0, 0, 8.74},
                {0, 0, 8.74},
                {0, 0, 8.74}
            },
            // torque constant
//            0.025126582278481014,
            {0.11697849233439939, 0.11697849233439939, 0.11697849233439939, 0.11697849233439939},
            // T, RPM time constant
            {0.03, 0.03, 0.03, 0.03},
            {0.03, 0.03, 0.03, 0.03},
            // mass vehicle
            2.000,
            // gravity
            {0, 0, -9.81},
            // J
            {
                {
                    0.0216666666666666,
                    0.0000000000000000000000000000000000000000,
                    0.0000000000000000000000000000000000000000
                },
                {
                    0.0000000000000000000000000000000000000000,
                    0.0216666666666666,
                    0.0000000000000000000000000000000000000000
                },
                {
                    0.0000000000000000000000000000000000000000,
                    0.0000000000000000000000000000000000000000,
                    0.04
                }
            },
            // J_inv
            {
                {
                    46.153846153846295,
                    0.0000000000000000000000000000000000000000,
                    0.0000000000000000000000000000000000000000
                },
                {
                    0.0000000000000000000000000000000000000000,
                    46.153846153846295,
                    0.0000000000000000000000000000000000000000
                },
                {
                    0.0000000000000000000000000000000000000000,
                    0.0000000000000000000000000000000000000000,
                    25
                }
            },
            // hovering throttle (julia): sqrt((mass * 9.81/4 - thrust_curve[1])/thrust_curve[3]),
            0.749141384950337, //julia sqrt((mass * 9.81/4 - thrust_curve[1])/thrust_curve[3])
            // action limit
            {0.0, 1.0},
        };

    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif