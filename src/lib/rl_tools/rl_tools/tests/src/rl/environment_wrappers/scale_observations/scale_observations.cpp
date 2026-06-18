#include <rl_tools/operations/cpu.h>
#include <gtest/gtest.h>
#include <rl_tools/rl/environments/pendulum/operations_generic.h>
#include <rl_tools/rl/environment_wrappers/scale_observations/operations_generic.h>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using T = double;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;

using PENDULUM_SPEC = rlt::rl::environments::pendulum::Specification<T, TI, rlt::rl::environments::pendulum::DefaultParameters<T>>;
using ENVIRONMENT = rlt::rl::environments::Pendulum<PENDULUM_SPEC>;
TEST(RL_TOOLS_RL_ENVIRONMENT_WRAPPERS_SCALE_OBSERVATIONS, IDENTITY_SCALING){
    using SCALE_OBSERVATIONS_WRAPPER_SPEC = rlt::rl::environment_wrappers::scale_observations::Specification<TYPE_POLICY, TI>;
    using WRAPPED_ENVIRONMENT = rlt::rl::environment_wrappers::ScaleObservations<SCALE_OBSERVATIONS_WRAPPER_SPEC, ENVIRONMENT>;
    DEVICE device;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters parameters;
    ENVIRONMENT::State state;
    WRAPPED_ENVIRONMENT wrapped_env;
    WRAPPED_ENVIRONMENT::State wrapped_state;
    WRAPPED_ENVIRONMENT::Parameters wrapped_parameters;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    auto wrapped_rng = rng;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM, false>> observation;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, WRAPPED_ENVIRONMENT::Observation::DIM, false>> wrapped_observation;
    rlt::initial_parameters(device, env, parameters);
    rlt::initial_state(device, env, parameters, state);
    rlt::initial_state(device, wrapped_env, parameters, wrapped_state);
    rlt::observe(device, env, parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
    rlt::observe(device, wrapped_env, wrapped_parameters, wrapped_state, typename WRAPPED_ENVIRONMENT::Observation{}, wrapped_observation, wrapped_rng);
    T diff = rlt::abs_diff(device, observation, wrapped_observation);
    ASSERT_LT(diff, 1e-10);
}

struct SCALE_BY_10_OBSERVATIONS_WRAPPER_SPEC: rlt::rl::environment_wrappers::scale_observations::Specification<TYPE_POLICY, TI>{
    static constexpr T SCALE = 10;
};

TEST(RL_TOOLS_RL_ENVIRONMENT_WRAPPERS_SCALE_OBSERVATIONS, ACTUAL_SCALING){
    using WRAPPED_ENVIRONMENT = rlt::rl::environment_wrappers::ScaleObservations<SCALE_BY_10_OBSERVATIONS_WRAPPER_SPEC, ENVIRONMENT>;
    DEVICE device;
    ENVIRONMENT env;
    ENVIRONMENT::Parameters parameters;
    ENVIRONMENT::State state;
    WRAPPED_ENVIRONMENT wrapped_env;
    WRAPPED_ENVIRONMENT::State wrapped_state;
    DEVICE::SPEC::RANDOM::ENGINE<> rng, wrapped_rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    wrapped_rng = rng;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::Observation::DIM, false>> observation;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, WRAPPED_ENVIRONMENT::Observation::DIM, false>> wrapped_observation;
    rlt::initial_state(device, env, parameters, state);
    rlt::initial_state(device, wrapped_env, parameters, wrapped_state);
    rlt::observe(device, env, parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);
    rlt::observe(device, wrapped_env, parameters, wrapped_state, typename WRAPPED_ENVIRONMENT::Observation{}, wrapped_observation, wrapped_rng);
    rlt::multiply_all(device, wrapped_observation, 1/SCALE_BY_10_OBSERVATIONS_WRAPPER_SPEC::SCALE);
    T diff = rlt::abs_diff(device, observation, wrapped_observation);
    ASSERT_LT(diff, 1e-10);
}
