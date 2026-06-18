RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    namespace rl::components::on_policy_runner{
        template <typename T_SPEC>
        struct CollectionEvaluationBuffer{
            using SPEC = T_SPEC;
            using T = typename SPEC::T;
            using TI = typename SPEC::TI;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::Observation::DIM>> observations;
            typename SPEC::CONTAINER_TYPE_TAG::template type<matrix::Specification<T, TI, SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::ACTION_DIM>> actions;
        };
    }
    template <typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, rl::components::on_policy_runner::CollectionEvaluationBuffer<SPEC>& buffer){
        malloc(device, buffer.observations);
        malloc(device, buffer.actions);
    }
    template <typename DEVICE, typename SPEC>
    void free(DEVICE& device, rl::components::on_policy_runner::CollectionEvaluationBuffer<SPEC>& buffer){
        free(device, buffer.observations);
        free(device, buffer.actions);
    }
    template <typename DEVICE, typename DEVICE_EVALUATION, typename DATASET_SPEC, typename ACTOR, typename ACTOR_EVALUATION, typename RNG, typename RNG_EVAL> // todo: make this not PPO but general policy with output distribution
    void collect_hybrid(DEVICE& device, DEVICE_EVALUATION& device_evaluation, rl::components::on_policy_runner::Dataset<DATASET_SPEC>& dataset, rl::components::OnPolicyRunner<typename DATASET_SPEC::SPEC>& runner, ACTOR& actor, ACTOR_EVALUATION& actor_evaluation, typename ACTOR_EVALUATION::template Buffer<DATASET_SPEC::SPEC::N_ENVIRONMENTS>& policy_eval_buffers, rl::components::on_policy_runner::CollectionEvaluationBuffer<typename DATASET_SPEC::SPEC> evaluation_buffer, rl::components::on_policy_runner::CollectionEvaluationBuffer<typename DATASET_SPEC::SPEC>& evaluation_buffer_evaluation, RNG& rng, RNG_EVAL& rng_evaluation){
#ifdef RL_TOOLS_DEBUG_RL_COMPONENTS_ON_POLICY_RUNNER_CHECK_INIT
        utils::assert_exit(device, runner.initialized, "rl::components::on_policy_runner::collect: runner not initialized");
#endif
        using SPEC = typename DATASET_SPEC::SPEC;
        using BUFFER = rl::components::on_policy_runner::Dataset<SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
//        TI prologue_time = 0;
//        TI copy_observations_time = 0;
//        TI evaluate_time = 0;
//        TI copy_back_time = 0;
//        TI epilogue_time = 0;
        for(TI step_i = 0; step_i < DATASET_SPEC::STEPS_PER_ENV; step_i++){
            auto actions_mean            = view(device, dataset.actions_mean               , matrix::ViewSpec<SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::ACTION_DIM>()                , step_i*SPEC::N_ENVIRONMENTS, 0);
            auto actions                 = view(device, dataset.actions                    , matrix::ViewSpec<SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::ACTION_DIM>()                , step_i*SPEC::N_ENVIRONMENTS, 0);
            auto observations            = view(device, dataset.observations               , matrix::ViewSpec<SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::Observation::DIM>()          , step_i*SPEC::N_ENVIRONMENTS, 0);
            auto observations_privileged = view(device, dataset.all_observations_privileged, matrix::ViewSpec<SPEC::N_ENVIRONMENTS, SPEC::ENVIRONMENT::ObservationPrivileged::DIM>(), step_i*SPEC::N_ENVIRONMENTS, 0);

            {
//                auto start = std::chrono::high_resolution_clock::now();
                rl::components::on_policy_runner::prologue(device, observations_privileged, observations, runner, rng, step_i);
//                auto end = std::chrono::high_resolution_clock::now();
//                prologue_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
            {
//                auto start = std::chrono::high_resolution_clock::now();
                copy(device, device_evaluation, observations, evaluation_buffer_evaluation.observations);
//                auto end = std::chrono::high_resolution_clock::now();
//                copy_observations_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
            {
//                auto start = std::chrono::high_resolution_clock::now();
                evaluate(device_evaluation, actor_evaluation, evaluation_buffer_evaluation.observations, evaluation_buffer_evaluation.actions, policy_eval_buffers, rng, rng_evaluation);
                cudaDeviceSynchronize();
//                auto end = std::chrono::high_resolution_clock::now();
//                evaluate_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
            {

//                auto start = std::chrono::high_resolution_clock::now();
                copy(device_evaluation, device, evaluation_buffer_evaluation.actions, evaluation_buffer.actions);
                copy(device, device, evaluation_buffer.actions, actions_mean);
//                auto end = std::chrono::high_resolution_clock::now();
//                copy_back_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
            {
//                auto start = std::chrono::high_resolution_clock::now();
                auto& last_layer = get_last_layer(actor);
                rl::components::on_policy_runner::epilogue(device, dataset, runner, actions_mean, actions, last_layer.log_std.parameters, rng, step_i);
//                auto end = std::chrono::high_resolution_clock::now();
//                epilogue_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            }
        }
//        std::cout << "prologue_time: " << prologue_time << std::endl;
//        std::cout << "copy_observations_time: " << copy_observations_time << std::endl;
//        std::cout << "evaluate_time: " << evaluate_time << std::endl;
//        std::cout << "copy_back_time: " << copy_back_time << std::endl;
//        std::cout << "epilogue_time: " << epilogue_time << std::endl;

        // final observation
        for(TI env_i = 0; env_i < SPEC::N_ENVIRONMENTS; env_i++){
            auto& env = get(runner.environments, 0, env_i);
            auto& env_parameters = get(runner.env_parameters, 0, env_i);
            auto& state = get(runner.states, 0, env_i);
            TI row_i = DATASET_SPEC::STEPS_PER_ENV * SPEC::N_ENVIRONMENTS + env_i;
            auto observation = row(device, dataset.all_observations_privileged, row_i);
            observe(device, env, env_parameters, state, typename SPEC::ENVIRONMENT::ObservationPrivileged{}, observation, rng);
//            auto observation = row(device, dataset.all_observations_normalized, row_i);
//            normalize(device, observations_mean, observations_std, observation, observation_normalized);
        }
        runner.step += SPEC::N_ENVIRONMENTS * DATASET_SPEC::STEPS_PER_ENV;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
