#include <rl_tools/rl/environments/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments::dummy {
    template <typename T_T, typename T_TI>
    struct Specification{
        using T = T_T;
        using TI = T_TI;
    };

    template <typename TI>
    struct Observation{
        static constexpr TI DIM = 2;
    };

    struct Parameters{};

    template <typename T, typename TI>
    struct State{
        static constexpr TI DIM = 1;
        TI state;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::environments{
    template <typename T_SPEC>
    struct Dummy: Environment<typename T_SPEC::T, typename T_SPEC::TI>{
        using SPEC = T_SPEC;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using State = dummy::State<T, TI>;
        using Parameters = dummy::Parameters;
        using Observation = dummy::Observation<TI>;
        using ObservationPrivileged = Observation;
        static constexpr TI N_AGENTS = 1; // single agent
        static constexpr TI ACTION_DIM = 1;
        static constexpr TI EPISODE_STEP_LIMIT = 200;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(DEVICE& device, const rl::environments::Dummy<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(DEVICE& device, const rl::environments::Dummy<SPEC>& env){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(DEVICE& device, const rl::environments::Dummy<SPEC>& env){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, RNG& rng){ }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, typename rl::environments::Dummy<SPEC>::State& state, RNG& rng){
        state.state = 0;
    }
    template<typename DEVICE, typename SPEC>
    static void initial_state(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, typename rl::environments::Dummy<SPEC>::State& state){
        state.state = 0;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT typename SPEC::T step(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, const typename rl::environments::Dummy<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename rl::environments::Dummy<SPEC>::State& next_state, RNG& rng) {
        next_state.state = state.state + 1;
        return 1;
    }
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, const typename rl::environments::Dummy<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename rl::environments::Dummy<SPEC>::State& next_state, RNG& rng){
        return state.state * 0.1;
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(DEVICE& device, const rl::environments::Dummy<SPEC>& env, const typename rl::environments::Dummy<SPEC>::Parameters& parameters, const typename rl::environments::Dummy<SPEC>::State& state, const typename rl::environments::dummy::Observation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 2);
        typedef typename SPEC::T T;
        set(observation, 0, 0, state.state);
        set(observation, 0, 1, state.state * state.state);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(DEVICE& device, const rl::environments::Dummy<SPEC>& env, typename rl::environments::Dummy<SPEC>::Parameters& parameters, const typename rl::environments::Dummy<SPEC>::State state, RNG& rng){
        using T = typename SPEC::T;
        return random::uniform_real_distribution(device.random, (T)0, (T)1, rng) > 0.9;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
