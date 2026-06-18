#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_EVALUATION_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_EVALUATION_STATE_H

#include "../../../../rl/utils/evaluation/evaluation.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::evaluation{
    template<typename T_CONFIG, typename T_NEXT = typename T_CONFIG::NEXT::template State<typename T_CONFIG::NEXT>>
    struct State: T_NEXT {
        using CONFIG = T_CONFIG;
        using NEXT = T_NEXT;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        rl::utils::evaluation::Buffer<rl::utils::evaluation::BufferSpecification<typename CONFIG::EVALUATION_SPEC, CONFIG::DYNAMIC_ALLOCATION>> evaluation_buffer;
        using RESULT_TYPE = rl::utils::evaluation::Result<typename CONFIG::EVALUATION_RESULT_SPEC>;
        Matrix<matrix::Specification<RESULT_TYPE, TI, 1, CONFIG::EVALUATION_PARAMETERS::N_EVALUATIONS, NEXT::CONFIG::DYNAMIC_ALLOCATION>> evaluation_results;
        typename CONFIG::RNG rng_eval, rng_eval_on_demand;
        Mode<typename CONFIG::EVALUATION_PARAMETERS::EVALUATION_MODE> evaluation_mode;
        using EVALUATION_ACTOR_TYPE_BATCH_SIZE = typename CONFIG::NN::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, CONFIG::EVALUATION_PARAMETERS::NUM_EVALUATION_EPISODES>;
        using EVALUATION_ACTOR_TYPE = typename EVALUATION_ACTOR_TYPE_BATCH_SIZE::template CHANGE_CAPABILITY<nn::capability::Forward<NEXT::CONFIG::DYNAMIC_ALLOCATION>>;
        typename EVALUATION_ACTOR_TYPE::template State<CONFIG::DYNAMIC_ALLOCATION> actor_deterministic_evaluation_state;
        typename EVALUATION_ACTOR_TYPE::template Buffer<NEXT::CONFIG::DYNAMIC_ALLOCATION> actor_deterministic_evaluation_buffers;
        typename NEXT::CONFIG::ENVIRONMENT_EVALUATION env_eval;
        typename NEXT::CONFIG::ENVIRONMENT_EVALUATION::Parameters env_eval_parameters;
        typename CONFIG::UI ui;
        bool evaluate_this_step = false;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




