#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_CAR_CAR_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_CAR_CAR_H

#include "../../../math/operations_generic.h"
#include "../environments.h"


RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::car {
    template <typename T>
    struct Tire{
        T B;
        T C;
        T D;
    };
    template <typename T>
    struct Parameters{
        T g   = 9.81;
        T m   = 0.041;
        T I   = 27.8e-6;
        T lf  = 0.029;
        T lr  = 0.033;
        Tire<T> tf = {2.5790, 1.2000, 0.1920};
        Tire<T> tr = {3.3852, 1.2691, 0.1737};
        T cm  = 0.287;
        T cr0 = 0.0;
        T cr2 = 0.00035;
        T vt  = 0.01;
        T dt  = 0.01;
    };

    template <typename T, typename TI, TI T_HEIGHT, TI T_WIDTH, TI T_TRACK_SCALE>
    struct ParametersTrack: Parameters<T>{
        static constexpr TI HEIGHT = T_HEIGHT;
        static constexpr TI WIDTH = T_WIDTH;
        static constexpr T TRACK_SCALE = T_TRACK_SCALE / 1000.0;
        bool track[HEIGHT][WIDTH];
    };

    template <typename T_T, typename T_TI, typename T_PARAMETERS = car::Parameters<T_T>>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
        using PARAMETERS = T_PARAMETERS;
    };

    template <typename T_T, typename T_TI, T_TI T_HEIGHT, T_TI T_WIDTH, T_TI T_TRACK_SCALE>
    struct SpecificationTrack: Specification<T_T, T_TI>{
        using T = T_T;
        using TI = T_TI;
        static constexpr T_TI HEIGHT = T_HEIGHT;
        static constexpr T_TI WIDTH = T_WIDTH;
        static constexpr T TRACK_SCALE = T_TRACK_SCALE/1000.0;
        static constexpr T BOUND_X_LOWER = -(T)WIDTH * TRACK_SCALE / 2.0;
        static constexpr T BOUND_Y_LOWER = -(T)HEIGHT * TRACK_SCALE / 2.0;
        static constexpr T BOUND_X_UPPER = (T)WIDTH * TRACK_SCALE / 2.0;
        static constexpr T BOUND_Y_UPPER = (T)HEIGHT * TRACK_SCALE / 2.0;
        using PARAMETERS = ParametersTrack<T, TI, HEIGHT, WIDTH, T_TRACK_SCALE>;
    };

    template <typename T, typename TI>
    struct State{
        static constexpr TI DIM = 6;
        T x;
        T y;
        T mu;
        T vx;
        T vy;
        T omega;
    };
    template <typename TI>
    struct ObservationCar{
        static constexpr TI DIM = 6;
    };
    template <typename TI>
    struct ObservationCarTrack{
        static constexpr TI DIM = 6 + 3;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_SPEC>
    struct Car: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = car::State<T, TI>;
        using Observation = car::ObservationCar<TI>;
        using ObservationPrivileged = Observation;
        static constexpr TI ACTION_DIM = 2;
        using Parameters = typename SPEC::PARAMETERS;
    };

    template <typename T_SPEC>
    struct CarTrack: Car<T_SPEC>{
        using TI = typename T_SPEC::TI;
        using Observation = car::ObservationCarTrack<TI>;
        using ObservationPrivileged = Observation;
        bool initialized = false;
        bool track[T_SPEC::PARAMETERS::HEIGHT][T_SPEC::PARAMETERS::WIDTH];
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END







#endif
