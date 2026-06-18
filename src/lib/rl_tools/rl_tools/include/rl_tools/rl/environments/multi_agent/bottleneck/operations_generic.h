#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_OPERATIONS_GENERIC_H
#include "bottleneck.h"
#include "../../operations_generic.h"
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::environments::multi_agent::bottleneck {
        template <typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T f_mod_python(const DEVICE& dev, T a, T b){
            return a - b * math::floor(dev.math, a / b);
        }

        template <typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT T angle_normalize(const DEVICE& dev, T x){
            return f_mod_python(dev, (x + math::PI<T>), (2 * math::PI<T>)) - math::PI<T>;
        }
    }
    namespace rl::environments::multi_agent::bottleneck{
        template<typename DEVICE, typename SPEC, typename T, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool check_collision_between_agents(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::bottleneck::AgentState<T, TI> agent_state_a, const typename rl::environments::multi_agent::bottleneck::AgentState<T, TI> agent_state_b){
            T dx = agent_state_a.position[0] - agent_state_b.position[0];
            T dy = agent_state_a.position[1] - agent_state_b.position[1];
            T d = math::sqrt(device.math, dx * dx + dy * dy + 1e-6);
            if(d < SPEC::PARAMETERS::AGENT_DIAMETER){
                return true;
            }
            return false;
        }
        template<typename DEVICE, typename SPEC, typename T, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool check_collision_with_arena_wall(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::bottleneck::AgentState<T, TI> agent_state){
            if(agent_state.position[0] < SPEC::PARAMETERS::AGENT_DIAMETER/2){
                return true;
            }
            if(agent_state.position[0] > SPEC::PARAMETERS::ARENA_WIDTH - SPEC::PARAMETERS::AGENT_DIAMETER/2){
                return true;
            }
            if(agent_state.position[1] < SPEC::PARAMETERS::AGENT_DIAMETER/2){
                return true;
            }
            if(agent_state.position[1] > SPEC::PARAMETERS::ARENA_HEIGHT - SPEC::PARAMETERS::AGENT_DIAMETER/2){
                return true;
            }
            return false;
        }
        template<typename DEVICE, typename SPEC, typename T, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT bool check_collision_with_center_wall(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::bottleneck::AgentState<T, TI> agent_state){
            // Check if agent is within the horizontal bounds of the center wall considering the agent diameter
            if (agent_state.position[0] > SPEC::PARAMETERS::ARENA_WIDTH / 2 - SPEC::PARAMETERS::BARRIER_WIDTH / 2 - SPEC::PARAMETERS::AGENT_DIAMETER / 2 &&
                agent_state.position[0] < SPEC::PARAMETERS::ARENA_WIDTH / 2 + SPEC::PARAMETERS::BARRIER_WIDTH / 2 + SPEC::PARAMETERS::AGENT_DIAMETER / 2) {
                // Check if agent is within the vertical bounds of the center wall excluding the bottleneck considering the agent diameter
                if (agent_state.position[1] < SPEC::PARAMETERS::BOTTLENECK_POSITION - SPEC::PARAMETERS::BOTTLENECK_WIDTH / 2 + SPEC::PARAMETERS::AGENT_DIAMETER / 2 ||
                    agent_state.position[1] > SPEC::PARAMETERS::BOTTLENECK_POSITION + SPEC::PARAMETERS::BOTTLENECK_WIDTH / 2 - SPEC::PARAMETERS::AGENT_DIAMETER / 2) {
                    return true;
                }
            }
            return false;
        }

        template <typename T>
        struct Ray {
            T origin[2];
            T direction[2];
        };

        template<typename DEVICE, typename SPEC, typename T, typename TI>
        RL_TOOLS_FUNCTION_PLACEMENT Intersection<T> intersects(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::bottleneck::AgentState<T, TI>& agent_state, Ray<T> ray) {
            Intersection<T> result;
            result.intersects = false;

            T dx = ray.origin[0] - agent_state.position[0];
            T dy = ray.origin[1] - agent_state.position[1];
            T radius = SPEC::PARAMETERS::AGENT_DIAMETER / 2;

            T a = ray.direction[0] * ray.direction[0] + ray.direction[1] * ray.direction[1] + 1e-6;
            T b = 2 * (ray.direction[0] * dx + ray.direction[1] * dy);
            T c = dx * dx + dy * dy - radius * radius;

            T discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                return result; // No intersection
            } else {
                T t1 = (-b - math::sqrt(device.math, discriminant + 1e-6)) / (2 * a);
                T t2 = (-b + math::sqrt(device.math, discriminant + 1e-6)) / (2 * a);
                if (t1 >= 0) {
                    result.intersects = true;
                    result.point[0] = ray.origin[0] + t1 * ray.direction[0];
                    result.point[1] = ray.origin[1] + t1 * ray.direction[1];
                    result.distance = t1;
                    return result; // Intersection at t1
                }
                if (t2 >= 0) {
                    result.intersects = true;
                    result.point[0] = ray.origin[0] + t2 * ray.direction[0];
                    result.point[1] = ray.origin[1] + t2 * ray.direction[1];
                    result.distance = t2;
                    return result; // Intersection at t2
                }
            }
            return result; // No intersection
        }

        template <typename T>
        struct AxisAlignedRectangle {
            T min[2];
            T max[2];
        };

        template<typename DEVICE, typename T>
        RL_TOOLS_FUNCTION_PLACEMENT Intersection<T> intersects(DEVICE& device, AxisAlignedRectangle<T> rectangle, Ray<T> ray) {
            Intersection<T> result;
            result.intersects = false;

            // disgusting hack to prevent divison by zero
            if(ray.direction[0] < 1e-6 && ray.direction[0] > -1e-6){
                if(ray.direction[0] > 0){
                    ray.direction[0] = 1e-6;
                }
                else{
                    ray.direction[0] = -1e-6;
                }
            }
            if(ray.direction[1] < 1e-6 && ray.direction[1] > -1e-6){
                if(ray.direction[1] > 0){
                    ray.direction[1] = 1e-6;
                }
                else{
                    ray.direction[1] = -1e-6;
                }
            }

            T tmin = (rectangle.min[0] - ray.origin[0]) / ray.direction[0];
            T tmax = (rectangle.max[0] - ray.origin[0]) / ray.direction[0];

            if (tmin > tmax) std::swap(tmin, tmax);

            T tymin = (rectangle.min[1] - ray.origin[1]) / ray.direction[1];
            T tymax = (rectangle.max[1] - ray.origin[1]) / ray.direction[1];

            if (tymin > tymax) std::swap(tymin, tymax);

            if ((tmin > tymax) || (tymin > tmax))
                return result;

            if (tymin > tmin)
                tmin = tymin;

            if (tymax < tmax)
                tmax = tymax;

            if (tmin < 0 && tmax < 0)
                return result;

            T t = (tmin < 0) ? tmax : tmin;

            result.intersects = true;
            result.point[0] = ray.origin[0] + t * ray.direction[0];
            result.point[1] = ray.origin[1] + t * ray.direction[1];
            result.distance = t;
            return result;
        }
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, RNG& rng){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void update_lidar(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state) {
        using ENV = rl::environments::multi_agent::Bottleneck<SPEC>;
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        using PARAMS = typename SPEC::PARAMETERS;
        using TI = typename DEVICE::index_t;
        // Lidar
        for(TI agent_i=0; agent_i < ENV::PARAMETERS::N_AGENTS; agent_i++) {
            auto &agent_next_state = state.agent_states[agent_i];
            if (!agent_next_state.dead) {
                for (TI lidar_i = 0; lidar_i < SPEC::PARAMETERS::LIDAR_RESOLUTION; lidar_i++) {
                    T angle = agent_next_state.orientation + (lidar_i - ((T)SPEC::PARAMETERS::LIDAR_RESOLUTION - 1) / (T)2) * SPEC::PARAMETERS::LIDAR_FOV / ((T)SPEC::PARAMETERS::LIDAR_RESOLUTION - 1);
                    T dx = math::cos(device.math, angle);
                    T dy = math::sin(device.math, angle);
                    T max_range = SPEC::PARAMETERS::LIDAR_RANGE;
                    T min_range = 0;
                    T range = max_range;
                    rl::environments::multi_agent::bottleneck::Ray<T> ray;
                    ray.origin[0] = agent_next_state.position[0];
                    ray.origin[1] = agent_next_state.position[1];
                    ray.direction[0] = dx;
                    ray.direction[1] = dy;
                    rl::environments::multi_agent::bottleneck::Intersection<T> min_intersection;
                    min_intersection.intersects = false;
                    for (TI other_agent_i = 0; other_agent_i < SPEC::PARAMETERS::N_AGENTS; other_agent_i++) {
                        if(agent_i != other_agent_i){
                            auto &other_agent_next_state = state.agent_states[other_agent_i];
                            auto intersection = rl::environments::multi_agent::bottleneck::intersects(device, env, parameters, other_agent_next_state, ray);
                            if(intersection.intersects && intersection.distance >= 0){
                                if(!min_intersection.intersects || (intersection.intersects && min_intersection.distance > intersection.distance)){
                                    min_intersection = intersection;
                                }
                            }
                        }
                    }
                    rl::environments::multi_agent::bottleneck::AxisAlignedRectangle<T> center_wall_upper, center_wall_lower, arena;
                    center_wall_upper.min[0] = SPEC::PARAMETERS::ARENA_WIDTH / 2 - SPEC::PARAMETERS::BARRIER_WIDTH / 2;
                    center_wall_upper.max[0] = SPEC::PARAMETERS::ARENA_WIDTH / 2 + SPEC::PARAMETERS::BARRIER_WIDTH / 2;
                    center_wall_upper.min[1] = 0;
                    center_wall_upper.max[1] = SPEC::PARAMETERS::BOTTLENECK_POSITION - SPEC::PARAMETERS::BOTTLENECK_WIDTH / 2;

                    center_wall_lower = center_wall_upper;
                    center_wall_lower.min[1] = SPEC::PARAMETERS::BOTTLENECK_POSITION + SPEC::PARAMETERS::BOTTLENECK_WIDTH / 2;
                    center_wall_lower.max[1] = SPEC::PARAMETERS::ARENA_HEIGHT;
                    auto intersection_upper = rl::environments::multi_agent::bottleneck::intersects(device, center_wall_upper, ray);
                    auto intersection_lower = rl::environments::multi_agent::bottleneck::intersects(device, center_wall_lower, ray);
                    if(intersection_upper.intersects && intersection_upper.distance >= 0){
                        if(!min_intersection.intersects || (intersection_upper.intersects && min_intersection.distance > intersection_upper.distance)){
                            min_intersection = intersection_upper;
                        }
                    }
                    if(intersection_lower.intersects && intersection_lower.distance >= 0){
                        if(!min_intersection.intersects || (intersection_lower.intersects && min_intersection.distance > intersection_lower.distance)){
                            min_intersection = intersection_lower;
                        }
                    }

                    arena.min[0] = 0;
                    arena.max[0] = SPEC::PARAMETERS::ARENA_WIDTH;
                    arena.min[1] = 0;
                    arena.max[1] = SPEC::PARAMETERS::ARENA_HEIGHT;
                    auto intersection_arena = rl::environments::multi_agent::bottleneck::intersects(device, arena, ray);
                    if(intersection_arena.intersects && intersection_arena.distance >= 0){
                        if(!min_intersection.intersects || (intersection_arena.intersects && min_intersection.distance > intersection_arena.distance)){
                            min_intersection = intersection_arena;
                        }
                    }
                    agent_next_state.lidar[lidar_i] = min_intersection;
                }
            }
        }
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        static_assert(SPEC::PARAMETERS::ARENA_WIDTH/2 - SPEC::PARAMETERS::AGENT_DIAMETER > SPEC::PARAMETERS::AGENT_DIAMETER, "Arena not wide enough for a single agent");
        static_assert((SPEC::PARAMETERS::ARENA_WIDTH/2 - 2*SPEC::PARAMETERS::AGENT_DIAMETER) * (SPEC::PARAMETERS::ARENA_HEIGHT - 2*SPEC::PARAMETERS::AGENT_DIAMETER) > SPEC::PARAMETERS::N_AGENTS * SPEC::PARAMETERS::AGENT_DIAMETER * SPEC::PARAMETERS::AGENT_DIAMETER/4 * 9, "Arena area not large enough for the number of agents");
        Matrix<matrix::Specification<bool, TI, 1, SPEC::PARAMETERS::N_AGENTS, false>> terminated_values;
        bool successfull = false;
        while(!successfull){
            successfull = true;
            for(TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++){
                auto& agent_state = state.agent_states[agent_i];
                bool illegal = true;
                agent_state.dead = false;
                for(TI try_i = 0; try_i < 100; try_i++){
                    if(SPEC::PARAMETERS::SPAWN_BOTH_SIDES && random::uniform_real_distribution(device.random, (T)0, (T)1, rng) > 0.5){
                        // spawn on the right
                        agent_state.position[0] = random::uniform_real_distribution(device.random, SPEC::PARAMETERS::ARENA_WIDTH / 2 + SPEC::PARAMETERS::BOTTLENECK_WIDTH / 2 + SPEC::PARAMETERS::AGENT_DIAMETER, SPEC::PARAMETERS::ARENA_WIDTH - SPEC::PARAMETERS::AGENT_DIAMETER, rng);
                    }
                    else{
                        // spawn on the left
                        agent_state.position[0] = random::uniform_real_distribution(device.random, SPEC::PARAMETERS::AGENT_DIAMETER, SPEC::PARAMETERS::ARENA_WIDTH / 2 - SPEC::PARAMETERS::AGENT_DIAMETER, rng);
                    }
                    agent_state.position[1] = random::uniform_real_distribution(device.random, SPEC::PARAMETERS::AGENT_DIAMETER, SPEC::PARAMETERS::ARENA_HEIGHT - SPEC::PARAMETERS::AGENT_DIAMETER, rng);
                    agent_state.orientation = random::uniform_real_distribution(device.random, -math::PI<T>, math::PI<T>, rng);
                    agent_state.velocity[0] = 0;
                    agent_state.velocity[1] = 0;
                    agent_state.angular_velocity = 0;
                    if(rl::environments::multi_agent::bottleneck::check_collision_with_arena_wall(device, env, parameters, agent_state)){
                        continue;
                    }
                    if(rl::environments::multi_agent::bottleneck::check_collision_with_center_wall(device, env, parameters, agent_state)){
                        continue;
                    }
                    bool agent_collision = false;
                    for(TI other_agent_i = 0; other_agent_i < agent_i; other_agent_i++){
                        if(rl::environments::multi_agent::bottleneck::check_collision_between_agents(device, env, parameters, agent_state, state.agent_states[other_agent_i])){
                            agent_collision = true;
                            break;
                        }
                    }
                    illegal = agent_collision;
                };
                if(illegal){
                    successfull = false;
                    break;
                }
            }
//            if(!successfull){
//                log(device, device.logger, "Failed to initialize agents, retrying...");
//            }
        };
        update_lidar(device, env, parameters, state);
    }
    template<typename DEVICE, typename SPEC>
    static void initial_state(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state){
        static_assert(SPEC::PARAMETERS::ARENA_WIDTH/2 - SPEC::PARAMETERS::AGENT_DIAMETER > SPEC::PARAMETERS::AGENT_DIAMETER, "Arena not wide enough for a single agent");
        static_assert((SPEC::PARAMETERS::ARENA_HEIGHT - SPEC::PARAMETERS::AGENT_DIAMETER) > SPEC::PARAMETERS::N_AGENTS * SPEC::PARAMETERS::AGENT_DIAMETER, "Arena not tall enough for initializing the agents on a line");
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        for(TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++){
            auto& agent_state = state.agent_states[agent_i];
            agent_state.position[0] = SPEC::PARAMETERS::ARENA_HEIGHT / 2;
            agent_state.position[1] = SPEC::PARAMETERS::AGENT_DIAMETER/2 + (agent_i) * SPEC::PARAMETERS::AGENT_DIAMETER * 1.5;
            agent_state.orientation = 0;
            agent_state.velocity[0] = 0;
            agent_state.velocity[1] = 0;
            agent_state.angular_velocity = 0;
            agent_state.dead = false;
        }
        update_lidar(device, env, parameters, state);
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::multi_agent::Bottleneck<SPEC>::State& next_state, RNG& rng) {
        using ENV = rl::environments::multi_agent::Bottleneck<SPEC>;
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == ENV::ACTION_DIM);
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        using PARAMS = typename SPEC::PARAMETERS;
        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;

#ifdef RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_CHECK_NAN
        utils::assert_exit(device, !is_nan(device, action), "Action is nan");
#endif

        for(TI agent_i=0; agent_i < N_AGENTS; agent_i++){
            auto& agent_state = state.agent_states[agent_i];
            auto& agent_next_state = next_state.agent_states[agent_i];
            if(!agent_state.dead){
                T acceleration_normalized = math::clamp(device.math, get(action, 0, agent_i * ENV::PER_AGENT_ACTION_DIM + 0), (T)-1, (T)1);
                T angular_acceleration_normalized = math::clamp(device.math, get(action, 0, agent_i * ENV::PER_AGENT_ACTION_DIM + 1), (T)-1, (T)1);
                T acceleration = PARAMS::AGENT_MAX_ACCELERATION * acceleration_normalized;
                T angular_acceleration = PARAMS::AGENT_MAX_ANGULAR_ACCELERATION * angular_acceleration_normalized;
                T dt = PARAMS::DT;
                T dx = agent_state.velocity[0] * dt;
                T dy = agent_state.velocity[1] * dt;
                T dtheta = agent_state.angular_velocity * dt;
                T new_x = agent_state.position[0] + dx;
                T new_y = agent_state.position[1] + dy;
                T new_theta = rl::environments::multi_agent::bottleneck::angle_normalize(device, agent_state.orientation + dtheta);
                T new_vx = agent_state.velocity[0] + acceleration * math::cos(device.math, new_theta) * dt;
                T new_vy = agent_state.velocity[1] + acceleration * math::sin(device.math, new_theta) * dt;
                T speed = math::sqrt(device.math, new_vx * new_vx + new_vy * new_vy + 1e-6);
                if(speed > PARAMS::AGENT_MAX_SPEED){
                    new_vx = new_vx / speed * PARAMS::AGENT_MAX_SPEED;
                    new_vy = new_vy / speed * PARAMS::AGENT_MAX_SPEED;
                }
                T new_omega = agent_state.angular_velocity + angular_acceleration * dt;
                new_omega = math::clamp(device.math, new_omega, -PARAMS::AGENT_MAX_ANGULAR_VELOCITY, PARAMS::AGENT_MAX_ANGULAR_VELOCITY);
                agent_next_state.position[0] = new_x;
                agent_next_state.position[1] = new_y;
                agent_next_state.orientation = new_theta;
                agent_next_state.velocity[0] = new_vx;
                agent_next_state.velocity[1] = new_vy;
                agent_next_state.angular_velocity = new_omega;
                agent_next_state.dead = false;
            }
            else{
                agent_next_state = agent_state;
            }
        }
        for(TI agent_i=0; agent_i < ENV::PARAMETERS::N_AGENTS; agent_i++){
            auto& agent_next_state = next_state.agent_states[agent_i];
            if(!agent_next_state.dead){
                if(rl::environments::multi_agent::bottleneck::check_collision_with_arena_wall(device, env, parameters, agent_next_state)){
                    agent_next_state.dead = true;
                }
                if(rl::environments::multi_agent::bottleneck::check_collision_with_center_wall(device, env, parameters, agent_next_state)){
                    agent_next_state.dead = true;
                }
                for(TI other_agent_i = 0; other_agent_i < SPEC::PARAMETERS::N_AGENTS; other_agent_i++){
                    if(other_agent_i != agent_i){
                        auto& other_agent_next_state = next_state.agent_states[other_agent_i];
                        if(rl::environments::multi_agent::bottleneck::check_collision_between_agents(device, env, parameters, agent_next_state, other_agent_next_state)){
                            agent_next_state.dead = true;
                            other_agent_next_state.dead = true;
                        }
                    }
                }
            }
        }

        update_lidar(device, env, parameters, next_state);


        return SPEC::PARAMETERS::DT;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T reward(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& next_state, RNG& rng){
        using ENV = rl::environments::multi_agent::Bottleneck<SPEC>;
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == ENV::ACTION_DIM);
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr T RIGHT_SIDE_REWARD = 0.01;
        T acc = 0;
        T death_penalty = 0;
        for(TI agent_i = 0; agent_i < ENV::PARAMETERS::N_AGENTS; agent_i++){
            auto& agent_state = next_state.agent_states[agent_i];
            T current_reward = 0;
            if(agent_state.dead){
                if(!state.agent_states[agent_i].dead){
                    death_penalty += -1; // one-time death penalty
                }
            }
            else{
                current_reward += agent_state.position[0] > SPEC::PARAMETERS::ARENA_WIDTH/2 ? RIGHT_SIDE_REWARD : 0;
                current_reward -= agent_state.angular_velocity * agent_state.angular_velocity * 0.001;
                T force = get(action, 0, agent_i*2+0);
                T torque = get(action, 0, agent_i*2+1);
                current_reward -= force*force * 0.001 + torque*torque * 0.001;
            }
            acc += math::max(device.math, (T)0, current_reward);
        }
#ifdef RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_CHECK_NAN
        utils::assert_exit(device, !math::is_nan(device.math, acc), "reward is nan");
#endif
        return death_penalty + math::max(device.math, (T)0, acc * 10);
    }
//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename REWARD_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT void reward(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& next_state, Matrix<REWARD_SPEC>& reward, RNG& rng){
//        set_all(device, reward, acc);
//    }

    template<typename DEVICE, typename SPEC, typename OBS_SPEC, typename OBS_PARAMETERS, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT  void observe(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, const rl::environments::multi_agent::bottleneck::Observation<OBS_PARAMETERS>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using OBS = rl::environments::multi_agent::bottleneck::Observation<OBS_PARAMETERS>;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == OBS::DIM);
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI PER_AGENT_OBS_DIM = OBS::PER_AGENT_DIM;
        for(TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++){
            auto& agent_state = state.agent_states[agent_i];
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 0, agent_state.position[0]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 1, agent_state.position[1]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 2, agent_state.orientation);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 3, agent_state.dead ? 0 : agent_state.velocity[0]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 4, agent_state.dead ? 0 : agent_state.velocity[1]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 5, agent_state.dead ? 0 : agent_state.angular_velocity);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 6, agent_state.dead ? -1 : 1);
            for(TI lidar_i = 0; lidar_i < SPEC::PARAMETERS::LIDAR_RESOLUTION; lidar_i++){
                set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 7 + lidar_i, agent_state.dead ? 0 : agent_state.lidar[lidar_i].distance);
            }
        }
#ifdef RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_CHECK_NAN
        utils::assert_exit(device, !is_nan(device, observation), "Observation is nan");
#endif
    }
    template<typename DEVICE, typename SPEC, typename OBS_SPEC, typename OBS_PARAMETERS, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void observe(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State& state, const rl::environments::multi_agent::bottleneck::ObservationPrivileged<OBS_PARAMETERS>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        using OBS = rl::environments::multi_agent::bottleneck::Observation<OBS_PARAMETERS>;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == OBS::DIM);
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        constexpr TI PER_AGENT_OBS_DIM = OBS::PER_AGENT_DIM;
        for(TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++){
            auto& agent_state = state.agent_states[agent_i];
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 0, agent_state.position[0]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 1, agent_state.position[1]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 2, agent_state.orientation);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 3, agent_state.dead ? 0 : agent_state.velocity[0]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 4, agent_state.dead ? 0 : agent_state.velocity[1]);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 5, agent_state.dead ? 0 : agent_state.angular_velocity);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 6, agent_state.dead ? -1 : 1);
            for(TI lidar_i = 0; lidar_i < SPEC::PARAMETERS::LIDAR_RESOLUTION; lidar_i++){
                set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 7 + lidar_i, agent_state.dead ? 0 : agent_state.lidar[lidar_i].distance);
            }
        }
#ifdef RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_BOTTLENECK_CHECK_NAN
        utils::assert_exit(device, !is_nan(device, observation), "ObservationPrivileged is nan");
#endif
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT bool terminated(DEVICE& device, const rl::environments::multi_agent::Bottleneck<SPEC>& env, const typename rl::environments::multi_agent::Bottleneck<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::Bottleneck<SPEC>::State state, RNG& rng){
        using T = typename SPEC::T;
        using TI = typename DEVICE::index_t;
        for(TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++){
            if(!state.agent_states[agent_i].dead){
                return false;
            }
        }
        return true; // all dead
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
