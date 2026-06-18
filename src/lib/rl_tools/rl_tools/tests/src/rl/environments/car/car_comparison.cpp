#include <rl_tools/operations/cpu.h>

#include <rl_tools/rl/environments/car/car.h>
#include <rl_tools/rl/environments/car/operations_generic.h>
namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;

#include <gtest/gtest.h>

TEST(RL_ENVIRONMENTS_CAR, COMPARISON){
    using DEVICE = rlt::devices::DefaultCPU;
    using T = double;
    using TI = typename DEVICE::index_t;
    using ENV_SPEC = rlt::rl::environments::car::Specification<T, TI>;
    using ENVIRONMENT = rlt::rl::environments::Car<ENV_SPEC>;

    DEVICE device;
    ENVIRONMENT env;
    DEVICE::SPEC::RANDOM::ENGINE<> rng;

    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    ENVIRONMENT::Parameters parameters;
    ENVIRONMENT::State state, next_state;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, ENVIRONMENT::ACTION_DIM>> a;
    rlt::malloc(device, a);

    {
        rlt::initial_parameters(device, env, parameters);
        rlt::initial_state(device, env, parameters, state);
        set(a, 0, 0, 0.11);
        set(a, 0, 1, -30.0/180.0*rlt::math::PI<T>);
        for(TI step_i=0; step_i < 100; step_i++){
            rlt::step(device, env, parameters, state, a, next_state, rng);
            state = next_state;
            std::cout << "step: " << step_i << " x: " << state.x << ", y: " << state.y << ", mu: " << state.mu << ", vx: " << state.vx << ", vy: " << state.vy << ", omega: " << state.omega << "\n";
        }
        ENVIRONMENT::State target_state = {0.11012406573455273, -0.2547549112167621, -2.1226171833457834, 0.5411147087810739, 0.06409534767749557, -3.1425915975367156};

        EXPECT_NEAR(state.x, target_state.x, 1e-15);
        EXPECT_NEAR(state.y, target_state.y, 1e-15);
        EXPECT_NEAR(state.mu, target_state.mu, 1e-15);
        EXPECT_NEAR(state.vx, target_state.vx, 1e-15);
        EXPECT_NEAR(state.vy, target_state.vy, 1e-15);
        EXPECT_NEAR(state.omega, target_state.omega, 1e-15);
    }

    {
        rlt::initial_parameters(device, env, parameters);
        rlt::initial_state(device, env, parameters, state);
        state.mu = 90/180.0*rlt::math::PI<T>;
        set(a, 0, 0, 0.11);
        set(a, 0, 1, -30.0/180.0*rlt::math::PI<T>);
        for(TI step_i=0; step_i < 100; step_i++){
            rlt::step(device, env, parameters, state, a, next_state, rng);
            state = next_state;
            std::cout << "step: " << step_i << " x: " << state.x << ", y: " << state.y << ", mu: " << state.mu << ", vx: " << state.vx << ", vy: " << state.vy << ", omega: " << state.omega << "\n";
        }
        ENVIRONMENT::State target_state = {0.2547549112167622, 0.11012406573455266, -0.5518208565508872, 0.5411147087810739, 0.06409534767749557, -3.1425915975367156};

        EXPECT_NEAR(state.x, target_state.x, 1e-15);
        EXPECT_NEAR(state.y, target_state.y, 1e-15);
        EXPECT_NEAR(state.mu, target_state.mu, 1e-15);
        EXPECT_NEAR(state.vx, target_state.vx, 1e-15);
        EXPECT_NEAR(state.vy, target_state.vy, 1e-15);
        EXPECT_NEAR(state.omega, target_state.omega, 1e-15);
    }

}