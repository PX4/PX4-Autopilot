#include "../../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_CRAZYFLIE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_L2F_PARAMETERS_DYNAMICS_CRAZYFLIE_H
#include "../../multirotor.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::l2f::parameters::dynamics{
    template<typename T, typename TI> // Crazyflie is a quadrotor
    constexpr Dynamics<T, TI, 4> crazyflie = {
            // Rotor positions
            {
                    {
                            0.028,
                            -0.028,
                            0
                    },
                    {
                            -0.028,
                            -0.028,
                            0
                    },
                    {
                            -0.028,
                            0.028,
                            0
                    },
                    {
                            0.028,
                            0.028,
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
                    {0, 0, +1},
                    {0, 0, -1},
                    {0, 0, +1},
            },
            // thrust constants
            {
                    // {0.0213, -0.0112, 0.1201},
                    // {0.0213, -0.0112, 0.1201},
                    // {0.0213, -0.0112, 0.1201},
                    // {0.0213, -0.0112, 0.1201}
                    // {0, 0, 0.1302},
                    // {0, 0, 0.1302},
                    // {0, 0, 0.1302},
                    // {0, 0, 0.1302}
                    {0.00352526, 0.01437313, 0.09223048},
                    {0.00352526, 0.01437313, 0.09223048},
                    {0.00352526, 0.01437313, 0.09223048},
                    {0.00352526, 0.01437313, 0.09223048},
            },
            // torque constant
            {4.665e-3, 4.665e-3, 4.665e-3, 4.665e-3},
            // T, RPM time constant
            { // rising: ~(0.040 - 0.080)s from manufacturer plot
                    0.05545454545454546,
                    0.05545454545454546,
                    0.05545454545454546,
                    0.05545454545454546
            },
            { // falling: ~0.398s from manufacturer plot
                    0.24939393939393945,
                    0.24939393939393945,
                    0.24939393939393945,
                    0.24939393939393945
            },
            // mass vehicle
            0.027 + 0.0017 + 0.0003 + 0.0016, // take-off-weight, sd card deck, sd card, optical flow deck (v2)
            // gravity
            {0, 0, -9.81},
            // J
            {
                    {
                            9.416556729130406e-06,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            9.644051701582312e-06,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            1.745951732253285e-05
                    }
            },
            // J_inv
            {
                    {
                            106195.93007988465,
                            0.0,
                            0.0
                    },
                    {
                            0.0,
                            103690.85846314249,
                            0.0
                    },
                    {
                            0.0,
                            0.0,
                            57275.35197719487
                    }
            },
            // hovering throttle (julia): sqrt((mass * 9.81/4 - thrust_curve[1])/thrust_curve[3]),
//            "hovering_throttle": 14475.809152959684,
            0.7261389721508553, // "hovering_throttle_relative"
            // action limit
            {0, 1},
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif