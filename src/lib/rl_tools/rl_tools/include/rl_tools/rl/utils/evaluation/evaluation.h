#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_UTILS_EVALUATION_EVALUATION_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_UTILS_EVALUATION_EVALUATION_H
/*
 * This file relies on the environments methods hence it should be included after the operations of the environments that it will be used with
 */
#include "../../../utils/generic/typing.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::utils::evaluation{
    template <typename T, typename TI, typename ENV_STATE>
    struct State{
        T episode_return = 0;
        TI episode_step = 0;

        ENV_STATE state;
    };
    template <typename T_TYPE_POLICY, typename T_TI, typename T_ENVIRONMENT, T_TI T_N_EPISODES, T_TI T_STEP_LIMIT, bool T_DETERMINISTIC_INITIAL_STATE=false>
    struct Specification{
        using TYPE_POLICY = T_TYPE_POLICY;
        using T = typename TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        using ENVIRONMENT = T_ENVIRONMENT;
        constexpr static TI N_EPISODES = T_N_EPISODES;
        constexpr static TI STEP_LIMIT = T_STEP_LIMIT;
        constexpr static bool DETERMINISTIC_INITIAL_STATE = T_DETERMINISTIC_INITIAL_STATE;
    };
    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
    struct DataSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_DYNAMIC_ALLOCATION;
    };
    template <typename T_SPEC>
    struct Data{
        using SPEC = typename T_SPEC::SPEC;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        using ENVIRONMENT = typename SPEC::ENVIRONMENT;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        Tensor<tensor::Specification<typename ENVIRONMENT::Parameters, TI, tensor::Shape<TI, SPEC::N_EPISODES>, DYNAMIC_ALLOCATION>> parameters;
        Tensor<tensor::Specification<bool, TI, tensor::Shape<TI, SPEC::N_EPISODES, SPEC::STEP_LIMIT>, DYNAMIC_ALLOCATION>> terminated;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SPEC::N_EPISODES, SPEC::STEP_LIMIT>, DYNAMIC_ALLOCATION>> rewards;
        Tensor<tensor::Specification<typename ENVIRONMENT::State, TI, tensor::Shape<TI, SPEC::N_EPISODES, SPEC::STEP_LIMIT>, DYNAMIC_ALLOCATION>> states;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SPEC::N_EPISODES, SPEC::STEP_LIMIT, ENVIRONMENT::ACTION_DIM>, DYNAMIC_ALLOCATION>> actions;
        Tensor<tensor::Specification<T, TI, tensor::Shape<TI, SPEC::N_EPISODES, SPEC::STEP_LIMIT>, DYNAMIC_ALLOCATION>> dt;
    };
    template <typename T_SPEC>
    struct NoData{};
    template <typename T_SPEC>
    struct Result{
        using SPEC = T_SPEC;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        using ENVIRONMENT = typename SPEC::ENVIRONMENT;
        constexpr static auto N_EPISODES = SPEC::N_EPISODES;
        constexpr static auto STEP_LIMIT = SPEC::STEP_LIMIT;
        T returns[N_EPISODES];
        T returns_mean;
        T returns_std;
        TI episode_length[N_EPISODES];
        T episode_length_mean;
        T episode_length_std;
        TI num_terminated;
        T share_terminated;
    };
    template <typename T_SPEC, bool T_DYNAMIC_ALLOCATION=true>
    struct BufferSpecification{
        using SPEC = T_SPEC;
        static constexpr bool DYNAMIC_ALLOCATION=T_DYNAMIC_ALLOCATION;
    };
    template <typename T_SPEC>
    struct Buffer{
        using SPEC = typename T_SPEC::SPEC;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using TI = typename SPEC::TI;
        using ENVIRONMENT = typename SPEC::ENVIRONMENT;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        Matrix<matrix::Specification<T, TI, SPEC::N_EPISODES, ENVIRONMENT::ACTION_DIM, DYNAMIC_ALLOCATION>> actions;
        Matrix<matrix::Specification<T, TI, SPEC::N_EPISODES, ENVIRONMENT::Observation::DIM, DYNAMIC_ALLOCATION>> observations;
    };
    template <typename T_SPEC, typename T_POLICY, bool T_DYNAMIC_ALLOCATION=true>
    struct PolicyBufferSpecification{
        using SPEC = T_SPEC;
        using POLICY = T_POLICY;
        static constexpr bool DYNAMIC_ALLOCATION=T_DYNAMIC_ALLOCATION;
    };
    template <typename T_SPEC>
    struct PolicyBuffer{
        using SPEC = typename T_SPEC::SPEC;
        static constexpr bool DYNAMIC_ALLOCATION = T_SPEC::DYNAMIC_ALLOCATION;
        using POLICY_ORIG = typename T_SPEC::POLICY;
        using POLICY_BATCH_SIZE = typename POLICY_ORIG::template CHANGE_BATCH_SIZE<typename SPEC::TI, SPEC::N_EPISODES>;
        using POLICY = typename POLICY_BATCH_SIZE::template CHANGE_CAPABILITY<nn::capability::Forward<DYNAMIC_ALLOCATION>>;
        POLICY policy;
        typename POLICY::template State<DYNAMIC_ALLOCATION> policy_state;
        typename POLICY::template Buffer<DYNAMIC_ALLOCATION> policy_evaluation_buffers;
        Buffer<BufferSpecification<SPEC, DYNAMIC_ALLOCATION>> buffer;
    };

}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif