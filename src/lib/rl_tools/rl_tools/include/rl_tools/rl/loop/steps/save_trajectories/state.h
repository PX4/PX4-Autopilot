#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_LOOP_STEPS_SAVE_TRAJECTORIES_STATE_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_LOOP_STEPS_SAVE_TRAJECTORIES_STATE_H

#include "../../../../rl/utils/evaluation/evaluation.h"
#include "../../../../utils/generic/typing.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::loop::steps::save_trajectories{
    template<typename T_CONFIG, typename T_NEXT = typename T_CONFIG::NEXT::template State<typename T_CONFIG::NEXT>>
    struct State: T_NEXT {
        using CONFIG = T_CONFIG;
        using NEXT = T_NEXT;
        using TYPE_POLICY = typename CONFIG::TYPE_POLICY;
        using TI = typename CONFIG::TI;
        rl::utils::evaluation::Buffer<rl::utils::evaluation::BufferSpecification<typename CONFIG::SAVE_TRAJECTORIES_SPEC, CONFIG::DYNAMIC_ALLOCATION>> save_trajectories_buffer;
        rl::utils::evaluation::Result<typename CONFIG::SAVE_TRAJECTORIES_SPEC> save_trajectories_result;
        template <typename SPEC>
        using DATA_TYPE = rl_tools::utils::typing::conditional_t<CONFIG::SAVE_TRAJECTORIES_PARAMETERS::SAVE_TRAJECTORIES, rl::utils::evaluation::Data<rl::utils::evaluation::DataSpecification<SPEC>>, rl::utils::evaluation::NoData<rl::utils::evaluation::DataSpecification<SPEC>>>;
        DATA_TYPE<typename CONFIG::SAVE_TRAJECTORIES_SPEC> save_trajectories_data;
        typename CONFIG::RNG rng_save_trajectories;
        using SAVE_TRAJECTORIES_ACTOR_TYPE_BATCH_SIZE = typename CONFIG::NN::ACTOR_TYPE::template CHANGE_BATCH_SIZE<TI, CONFIG::SAVE_TRAJECTORIES_PARAMETERS::NUM_EPISODES>;
        using SAVE_TRAJECTORIES_ACTOR_TYPE = typename SAVE_TRAJECTORIES_ACTOR_TYPE_BATCH_SIZE::template CHANGE_CAPABILITY<nn::capability::Forward<CONFIG::DYNAMIC_ALLOCATION>>;
        typename SAVE_TRAJECTORIES_ACTOR_TYPE::template State<CONFIG::DYNAMIC_ALLOCATION> actor_deterministic_save_trajectories_state;
        typename SAVE_TRAJECTORIES_ACTOR_TYPE::template Buffer<CONFIG::DYNAMIC_ALLOCATION> actor_deterministic_save_trajectories_buffers;
        typename CONFIG::UI ui_save_trajectories;
        bool save_trajectories_ui_written = false;
        bool save_trajectories_this_step = false;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif




