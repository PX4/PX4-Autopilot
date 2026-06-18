#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_BOTTLENECK_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_BOTTLENECK_H

#include "../../../../math/operations_generic.h"
#include "../environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::multi_agent::bottleneck {
    template <typename T_T, typename T_TI>
    struct DefaultParameters {
        using T = T_T;
        using TI = T_TI;
        static constexpr TI N_AGENTS = 2;
        static constexpr TI LIDAR_RESOLUTION = 5;
        static constexpr T LIDAR_FOV = math::PI<T> * 20/180; // in radians (0 to PI)
        static constexpr T LIDAR_RANGE = 3;
        static constexpr T DT = 0.05;
        static constexpr T ARENA_WIDTH = 20; // x: left to right, y: top to bottom (display coordinate system)
        static constexpr T ARENA_HEIGHT = 20;
        static constexpr T AGENT_DIAMETER = 1.5;
        static constexpr T AGENT_MAX_SPEED = 2;
        static constexpr T AGENT_MAX_ACCELERATION = 2;
        static constexpr T AGENT_MAX_ANGULAR_VELOCITY = 2;
        static constexpr T AGENT_MAX_ANGULAR_ACCELERATION = 2;
        static constexpr T BOTTLENECK_POSITION = 10; // y
        static constexpr T BOTTLENECK_WIDTH = 2;
        static constexpr T BARRIER_WIDTH = 0.5;
        static constexpr TI EPISODE_STEP_LIMIT = 200;
        static constexpr bool SPAWN_BOTH_SIDES = false;
    };
    template <typename T_PARAMETERS>
    struct Observation{
        using PARAMETERS = T_PARAMETERS;
        using T = typename PARAMETERS::T;
        using TI = typename PARAMETERS::TI;
        static constexpr TI PER_AGENT_DIM = 7 + PARAMETERS::LIDAR_RESOLUTION;
        static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_DIM;
    };
    template <typename T_PARAMETERS>
    struct ObservationPrivileged{
        using PARAMETERS = T_PARAMETERS;
        using T = typename PARAMETERS::T;
        using TI = typename PARAMETERS::TI;
        static constexpr TI PER_AGENT_DIM = 7 + PARAMETERS::LIDAR_RESOLUTION;
        static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_DIM;
    };
    template <typename T_TYPE_POLICY, typename T_TI, typename T_PARAMETERS = DefaultParameters<typename T_TYPE_POLICY::DEFAULT, T_TI>, typename T_OBSERVATION = Observation<T_PARAMETERS>, typename T_OBSERVATION_PRIVILEGED = ObservationPrivileged<T_PARAMETERS>>
    struct Specification{
        using T = typename T_TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        using OBSERVATION = T_OBSERVATION;
        using OBSERVATION_PRIVILEGED = T_OBSERVATION_PRIVILEGED;
        using PARAMETERS = T_PARAMETERS;
    };

    template <typename T>
    struct Intersection {
        bool intersects = false;
        T point[2];
        T distance;
    };

    template <typename T, typename TI>
    struct AgentState {
        T position[2];
        T orientation;
        T velocity[2];
        T angular_velocity;
        Intersection<T> lidar[DefaultParameters<T, TI>::LIDAR_RESOLUTION];
        bool dead;
    };

    template <typename SPEC>
    struct State{
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        AgentState<T, TI> agent_states[SPEC::PARAMETERS::N_AGENTS];
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::multi_agent{
    template <typename T_SPEC>
    struct Bottleneck: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using PARAMETERS = typename SPEC::PARAMETERS;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = multi_agent::bottleneck::State<SPEC>;
        using Parameters = typename SPEC::PARAMETERS;
        using Observation = typename SPEC::OBSERVATION;
        using ObservationPrivileged = typename SPEC::OBSERVATION_PRIVILEGED;
        static constexpr TI N_AGENTS = Parameters::N_AGENTS;
        static constexpr TI PER_AGENT_ACTION_DIM = 2; // linear and angular acceleration
        static constexpr TI ACTION_DIM = N_AGENTS * PER_AGENT_ACTION_DIM;
        static constexpr TI EPISODE_STEP_LIMIT = Parameters::EPISODE_STEP_LIMIT;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END







#endif
