#include "../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_OPERATIONS_GENERIC_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_NN_LAYERS_SAMPLE_AND_SQUASH_OPERATIONS_GENERIC_H
#include "../../../nn/activation_functions.h"
#include "../../../utils/generic/typing.h"
#include "../../../containers/matrix/matrix.h"

#include "../gru/layer.h"
#include "../gru/operations_generic.h"


#include "layer.h"

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::sample_and_squash::LayerForward<SPEC>& layer){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::sample_and_squash::LayerForward<SPEC>& layer){ }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<SPEC>& layer){
        malloc(device, static_cast<nn::layers::sample_and_squash::LayerForward<SPEC>&>(layer));
        malloc(device, layer.pre_squashing);
        malloc(device, layer.noise);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<SPEC>& layer){
        free(device, static_cast<nn::layers::sample_and_squash::LayerForward<SPEC>&>(layer));
        free(device, layer.pre_squashing);
        free(device, layer.noise);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer){
        malloc(device, static_cast<nn::layers::sample_and_squash::LayerBackward<SPEC>&>(layer));
        malloc(device, layer.log_alpha);
        malloc(device, layer.log_probabilities);
        malloc(device, layer.output);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer){
        free(device, static_cast<nn::layers::sample_and_squash::LayerBackward<SPEC>&>(layer));
        free(device, layer.log_alpha);
        free(device, layer.log_probabilities);
        free(device, layer.output);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::sample_and_squash::Buffer<SPEC>& buffer) {
        malloc(device, buffer.noise);
        malloc(device, buffer.d_log_alpha);
        malloc(device, buffer.log_probabilities);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::sample_and_squash::Buffer<SPEC>& buffer) {
        free(device, buffer.noise);
        free(device, buffer.d_log_alpha);
        free(device, buffer.log_probabilities);
    }
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void malloc(DEVICE& device, nn::layers::sample_and_squash::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::sample_and_squash::State& source, nn::layers::sample_and_squash::State& target){}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE>
    RL_TOOLS_FUNCTION_PLACEMENT void reset(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<SPEC>& layer, nn::layers::sample_and_squash::State& state, RNG&, Mode<MODE> mode = Mode<mode::Default<>>{}) { } // no-op
    template<typename DEVICE>
    RL_TOOLS_FUNCTION_PLACEMENT void free(DEVICE& device, nn::layers::sample_and_squash::State& state) { } // no-op
    template <typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, nn::layers::sample_and_squash::Buffer<SOURCE_SPEC>& source, nn::layers::sample_and_squash::Buffer<TARGET_SPEC>& target){
        copy(source_device, target_device, source.noise, target.noise);
        copy(source_device, target_device, source.log_probabilities, target.log_probabilities);
        copy(source_device, target_device, source.d_log_alpha, target.d_log_alpha);
    }
    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::sample_and_squash::LayerForward<SPEC>& layer, RNG& rng){ }
    template <typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void init_weights(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, RNG& rng){
        init_weights(device, static_cast<nn::layers::sample_and_squash::LayerForward<SPEC>&>(layer), rng);
        using LOG_ALPHA_T = typename decltype(layer.log_alpha.parameters)::SPEC::T;
        set_all(device, layer.log_alpha.parameters, static_cast<LOG_ALPHA_T>(math::log(typename DEVICE::SPEC::MATH{}, SPEC::PARAMETERS::ALPHA)));
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::sample_and_squash::LayerBackward<SPEC>& l) {
        set_all(device, l.pre_squashing, 0);
        set_all(device, l.noise, 0);
    }
    template <typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void reset_forward_state(DEVICE& device, rl_tools::nn::layers::sample_and_squash::LayerGradient<SPEC>& l) {
        reset_forward_state(device, (rl_tools::nn::layers::sample_and_squash::LayerBackward<SPEC>&) l);
        set_all(device, l.log_probabilities, 0);
        set_all(device, l.output, 0);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void zero_gradient(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer) {
        zero_gradient(device, layer.log_alpha);
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void update(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer){
        if constexpr(SPEC::PARAMETERS::ADAPTIVE_ALPHA && SPEC::PARAMETERS::UPDATE_ALPHA_WITH_ACTOR){
            update(device, layer.log_alpha, optimizer);
        }
    }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    RL_TOOLS_FUNCTION_PLACEMENT void _reset_optimizer_state(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, OPTIMIZER& optimizer) {
        if constexpr(SPEC::PARAMETERS::UPDATE_ALPHA_WITH_ACTOR){
            _reset_optimizer_state(device, layer.log_alpha, optimizer);
        }
    }

    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::sample_and_squash::LayerForward<SOURCE_SPEC>& source, nn::layers::sample_and_squash::LayerForward<TARGET_SPEC>& target){ }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::sample_and_squash::LayerBackward<SOURCE_SPEC>& source, nn::layers::sample_and_squash::LayerBackward<TARGET_SPEC>& target){
        copy(source_device, target_device, source.pre_squashing, target.pre_squashing);
        copy(source_device, target_device, source.noise, target.noise);
    }
    template<typename SOURCE_DEVICE, typename TARGET_DEVICE, typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT void copy(SOURCE_DEVICE& source_device, TARGET_DEVICE& target_device, const  nn::layers::sample_and_squash::LayerGradient<SOURCE_SPEC>& source, nn::layers::sample_and_squash::LayerGradient<TARGET_SPEC>& target){
        copy(source_device, target_device, static_cast<const nn::layers::sample_and_squash::LayerBackward<SOURCE_SPEC>&>(source), static_cast<nn::layers::sample_and_squash::LayerBackward<TARGET_SPEC>&>(target));
        copy(source_device, target_device, source.log_probabilities, target.log_probabilities);
        copy(source_device, target_device, source.output, target.output);
        copy(source_device, target_device, source.log_alpha, target.log_alpha);
    }
    template<typename DEVICE,typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SOURCE_SPEC::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<SOURCE_SPEC>& source, const nn::layers::sample_and_squash::LayerForward<TARGET_SPEC>& target) {
        return 0;
    }
    template<typename DEVICE,typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SOURCE_SPEC::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const nn::layers::sample_and_squash::LayerBackward<SOURCE_SPEC>& source, const nn::layers::sample_and_squash::LayerBackward<TARGET_SPEC>& target) {
        typename SOURCE_SPEC::TYPE_POLICY::DEFAULT acc = 0;
        acc += abs_diff(device, static_cast<const nn::layers::sample_and_squash::LayerForward<SOURCE_SPEC>&>(source), static_cast<const nn::layers::sample_and_squash::LayerForward<TARGET_SPEC>&>(target));
        acc += abs_diff(device, source.pre_squashing, target.pre_squashing);
        acc += abs_diff(device, source.noise, target.noise);
        return acc;
    }
    template<typename DEVICE,typename SOURCE_SPEC, typename TARGET_SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT typename SOURCE_SPEC::TYPE_POLICY::DEFAULT abs_diff(DEVICE& device, const nn::layers::sample_and_squash::LayerGradient<SOURCE_SPEC>& source, const nn::layers::sample_and_squash::LayerGradient<TARGET_SPEC>& target) {
        typename SOURCE_SPEC::TYPE_POLICY::DEFAULT acc = 0;
        acc += abs_diff(device, static_cast<const nn::layers::sample_and_squash::LayerBackward<SOURCE_SPEC>&>(source), static_cast<const nn::layers::sample_and_squash::LayerBackward<TARGET_SPEC>&>(target));
        acc += abs_diff(device, source.log_probabilities, target.log_probabilities);
        acc += abs_diff(device, source.output, target.output);
        acc += abs_diff(device, source.log_alpha, target.log_alpha);
        return acc;
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT void sample(DEVICE& device, nn::layers::sample_and_squash::Buffer<SPEC>& buffer, RNG& rng) {
        randn(device, buffer.noise, rng);
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_per_sample(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, typename DEVICE::index_t row_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        using PARAMETERS = typename SPEC::PARAMETERS;
        T log_prob = 0;
        for(TI col_i = 0; col_i < SPEC::DIM; col_i++){
            T mean = get(input, row_i, col_i);
            T log_std = get(input, row_i, SPEC::DIM + col_i);
            T log_std_clipped = math::clamp(device.math, log_std, (T)PARAMETERS::LOG_STD_LOWER_BOUND, (T)PARAMETERS::LOG_STD_UPPER_BOUND);
            T std = math::exp(device.math, log_std_clipped);
            T noise;
            if constexpr(mode::is<MODE, nn::layers::sample_and_squash::mode::ExternalNoise>){
                noise = get(buffer.noise, row_i, col_i);
            }
            else{
                if constexpr(mode::is<MODE, mode::Default> || mode::is<MODE, mode::Rollout>){
                    noise = random::normal_distribution::sample(device.random, (T)0, (T)1, rng);
                }
                else{
                    if constexpr(mode::is<MODE, mode::Evaluation>){
                        noise = 0;
                    }
                    else{
                        noise = 0;
                        utils::assert_exit(device.logger, false, "Invalid mode");
                    }
                }
            }
//                set(layer.noise, row_i, col_i, noise);
            T sample;
            if constexpr(mode::is<MODE, mode::Evaluation>){
                sample = mean;
            }
            else{
                sample = mean + noise * std;
            }
//                set(layer.pre_squashing, row_i, col_i, sample);
            T squashed = math::tanh(device.math, sample);

            set(output, row_i, col_i, squashed);
//                set(layer.output, row_i, col_i, squashed);
            T one_minus_square_plus_eps = (1-squashed*squashed + SPEC::PARAMETERS::LOG_PROBABILITY_EPSILON);
            log_prob += random::normal_distribution::log_prob(device.random, mean, log_std_clipped, sample) - math::log(typename DEVICE::SPEC::MATH{}, one_minus_square_plus_eps);
        }
        set(buffer.log_probabilities, 0, row_i, log_prob);
//            set(layer.log_probabilities, 0, row_i, log_prob);
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(INPUT_SPEC::COLS == 2*SPEC::DIM);
        static_assert(OUTPUT_SPEC::COLS == SPEC::DIM);
        static_assert(INPUT_SPEC::ROWS == OUTPUT_SPEC::ROWS);
        using TI = typename DEVICE::index_t;
        using PARAMETERS = typename SPEC::PARAMETERS;
        for(TI row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            evaluate_per_sample(device, layer, input, output, buffer, rng, row_i, mode);
        }
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        evaluate(device, layer, input, output, buffer, rng, mode);
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward_per_sample(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, typename DEVICE::index_t row_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        // copy of the evaluate but with the log_probabilities commented in
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        using PARAMETERS = typename SPEC::PARAMETERS;
        T log_prob = 0;
        for(TI col_i = 0; col_i < SPEC::DIM; col_i++){
            T mean = get(input, row_i, col_i);
            T log_std = get(input, row_i, SPEC::DIM + col_i);
            T log_std_clipped = math::clamp(device.math, log_std, (T)PARAMETERS::LOG_STD_LOWER_BOUND, (T)PARAMETERS::LOG_STD_UPPER_BOUND);
            T std = math::exp(device.math, log_std_clipped);
            T noise;
            if(row_i == 0){
                // add_scalar(device, device.logger, "actor_std", std, 10001);
            }
            if constexpr(mode::is<MODE, nn::layers::sample_and_squash::mode::ExternalNoise>){
                noise = get(buffer.noise, row_i, col_i);
            }
            else{
                if constexpr(mode::is<MODE, mode::Default> || mode::is<MODE, mode::Rollout>){
                    noise = random::normal_distribution::sample(device.random, (T)0, (T)1, rng);
                }
                else{
                    if constexpr(mode::is<MODE, mode::Evaluation>){
                        noise = 0;
                    }
                    else{
                        noise = 0;
                        utils::assert_exit(device.logger, false, "Invalid mode");
                    }
                }
            }
            set(layer.noise, row_i, col_i, noise);
            T sample;
            if constexpr(mode::is<MODE, mode::Evaluation>){
                sample = mean;
            }
            else{
                sample = mean + noise * std;
            }
            set(layer.pre_squashing, row_i, col_i, sample);
            T squashed = math::tanh(device.math, sample);

//            set(output, row_i, col_i, squashed);
            set(layer.output, row_i, col_i, squashed);
            T one_minus_square_plus_eps = (1-squashed*squashed + SPEC::PARAMETERS::LOG_PROBABILITY_EPSILON);
            log_prob += random::normal_distribution::log_prob(device.random, mean, log_std_clipped, sample) - math::log(typename DEVICE::SPEC::MATH{}, one_minus_square_plus_eps);
        }
        set(layer.log_probabilities, 0, row_i, log_prob);
    }
    template <typename DEVICE, typename SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        static_assert(INPUT_SPEC::COLS == 2*SPEC::DIM);
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        using PARAMETERS = typename SPEC::PARAMETERS;
        {
            // logging
            for(TI col_i = 0; col_i < SPEC::DIM; col_i++){
                T log_std = get(input, 0, SPEC::DIM + col_i);
                T log_std_clipped = math::clamp(device.math, log_std, (T)PARAMETERS::LOG_STD_LOWER_BOUND, (T)PARAMETERS::LOG_STD_UPPER_BOUND);
                T std = math::exp(device.math, log_std_clipped);
                add_scalar(device, device.logger, "actor_std", std, 10001);
            }
        }
        for(TI row_i = 0; row_i < INPUT_SPEC::ROWS; row_i++){
            forward_per_sample(device, layer, input, buffer, rng, row_i, mode);
        }
    }
    template<typename DEVICE, typename SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::sample_and_squash::LayerBackward<SPEC>& layer, const Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>&, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        utils::assert_exit(device, false, "Not implemented");
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>&, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        utils::assert_exit(device, false, "Not implemented");
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full_per_sample(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, typename SPEC::TYPE_POLICY::DEFAULT alpha, typename DEVICE::index_t batch_i, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
        using TI = typename DEVICE::index_t;
        constexpr TI ACTION_DIM = SPEC::DIM;
        using LAYER = nn::layers::sample_and_squash::LayerGradient<SPEC>;
        constexpr TI INTERNAL_BATCH_SIZE = LAYER::INTERNAL_BATCH_SIZE;
        constexpr TI BATCH_SIZE = LAYER::SPEC::INTERNAL_BATCH_SIZE;
/*
        Gradient of the loss function:
        mu, std = policy(observation)
        action_sample = gaussian::sample(mu, std)
        action = tanh(action_sample)
        action_prob = gaussian::prob(mu, std, action_sample) * | d/d_action tanh^{-1}(action) |
                    = gaussian::prob(mu, std, action_sample) * | (d/d_action_sample tanh(action_sample))^{-1} |
                    = gaussian::prob(mu, std, action_sample) * | (d/d_action_sample tanh(action_sample))|^{-1}
                    = gaussian::prob(mu, std, action_sample) * ((1-tanh(action_sample)^2))^{-1}
        action_log_prob = gaussian::log_prob(mu, std, action_sample) - log(1-tanh(action_sample)^2))
        actor_loss = alpha  * action_log_prob - min(Q_1, Q_2);
        d/d_mu _actor_loss = alpha * d/d_mu action_log_prob - d/d_mu min(Q_1, Q_2)
        d/d_mu action_log_prob = d/d_mu gaussian::log_prob(mu, std, action_sample) + d/d_action_sample gaussian::log_prob(mu, std, action_sample) * d/d_mu action_sample - 1/(1-tanh(action_sample)^2) * (-2*tanh(action_sample))*(1-tanh(action_sample)^2) * d/d_mu action_sample)
                               = d/d_mu gaussian::log_prob(mu, std, action_sample) + d/d_action_sample gaussian::log_prob(mu, std, action_sample) * d/d_mu action_sample + 2*tanh(action_sample)) * d/d_mu action_sample
        d/d_std action_log_prob = d/d_std gaussian::log_prob(mu, std, action_sample) + d/d_action_sample gaussian::log_prob(mu, std, action_sample) * d/d_std action_sample + 2*tanh(action_sample) * d/d_std action_sample
        d/d_mu action_sample = 1
        d/d_std action_sample = noise
        d/d_mu min(Q_1, Q_2) = d/d_action min(Q_1, Q_2) * d/d_mu action
        d/d_mu action = d/d_action_sample tanh(action_sample) * d/d_mu action_sample
*/
        if constexpr(mode::is<MODE, nn::layers::sample_and_squash::mode::DisableEntropy>){
            alpha = 0;
        }
        T entropy = 0;
        for(TI action_i = 0; action_i < ACTION_DIM; action_i++){
            T action = get(layer.output, batch_i, action_i); // tanh(action_sample)
            T d_mu = 0;
            T d_std = 0;
            T d_output_value = get(d_output, batch_i, action_i);
            T d_tanh_pre_activation = d_output_value * (1-action*action);
            d_mu = d_tanh_pre_activation;
            d_std = d_tanh_pre_activation * get(layer.noise, batch_i, action_i);
            T log_std_pre_clamp = get(input, batch_i, action_i + ACTION_DIM);
            T log_std_clamped = math::clamp(device.math, log_std_pre_clamp, (T)SPEC::PARAMETERS::LOG_STD_LOWER_BOUND, (T)SPEC::PARAMETERS::LOG_STD_UPPER_BOUND);
            T std = math::exp(typename DEVICE::SPEC::MATH{}, log_std_clamped);

            T d_log_std_clamped = std * d_std;

            T mu = get(input, batch_i, action_i);
            T action_sample = get(layer.pre_squashing, batch_i, action_i);
            T d_log_prob_d_mean = random::normal_distribution::d_log_prob_d_mean(device.random, mu, log_std_clamped, action_sample);
            T d_log_prob_d_sample = random::normal_distribution::d_log_prob_d_sample(device.random, mu, log_std_clamped, action_sample);
            // NOTE: The following needs to be divided by BATCH_SIZE (in contrast to the previous d_mu and d_std). d_mu and d_std are already taking into account the mean prior to the backward call of the critic. Thence the d_critic_X_input is already divided by BATCH_SIZE
            d_mu += alpha/INTERNAL_BATCH_SIZE * (d_log_prob_d_mean + d_log_prob_d_sample + 2*action);

            T noise = get(layer.noise, batch_i, action_i);
            T d_log_prob_d_log_std = random::normal_distribution::d_log_prob_d_log_std(device.random, mu, log_std_clamped, action_sample);
            d_log_std_clamped += alpha/BATCH_SIZE * (d_log_prob_d_log_std + d_log_prob_d_sample * noise * std + 2*action * noise * std);
            T d_log_std = log_std_pre_clamp < SPEC::PARAMETERS::LOG_STD_LOWER_BOUND || log_std_pre_clamp > SPEC::PARAMETERS::LOG_STD_UPPER_BOUND ? 0 : d_log_std_clamped;

            set(d_input, batch_i, action_i, d_mu);
            set(d_input, batch_i, action_i + ACTION_DIM, d_log_std);

            T one_minus_action_square_plus_eps = (1-action*action + SPEC::PARAMETERS::LOG_PROBABILITY_EPSILON);
            T action_log_prob = random::normal_distribution::log_prob(device.random, mu, log_std_clamped, action_sample) - math::log(typename DEVICE::SPEC::MATH{}, one_minus_action_square_plus_eps);
            entropy += -action_log_prob;
        }
        T d_alpha = entropy - SPEC::PARAMETERS::TARGET_ENTROPY;
        T d_log_alpha = alpha*d_alpha; // d_log_alpha
        set(buffer.d_log_alpha, 0, batch_i, d_log_alpha);
    }
    template<typename DEVICE, typename SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& layer, const Matrix<INPUT_SPEC>& input, Matrix<D_OUTPUT_SPEC>& d_output, Matrix<D_INPUT_SPEC>& d_input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Gradient>;
        using TI = typename DEVICE::index_t;
        using LAYER = nn::layers::sample_and_squash::LayerGradient<SPEC>;
        constexpr TI BATCH_SIZE = LAYER::SPEC::INTERNAL_BATCH_SIZE;
        constexpr TI INTERNAL_BATCH_SIZE = LAYER::INTERNAL_BATCH_SIZE;
        T log_alpha = get(device, layer.log_alpha.parameters, 0);
        T alpha = math::exp(typename DEVICE::SPEC::MATH{}, log_alpha);
        {
            // logging
            T entropy = 0;
            for(TI action_i = 0; action_i < SPEC::DIM; action_i++){
                T action = get(layer.output, 0, action_i);
                T mu = get(input, 0, action_i);
                T log_std_pre_clamp = get(input, 0, action_i + SPEC::DIM);
                T log_std_clamped = math::clamp(device.math, log_std_pre_clamp, (T)SPEC::PARAMETERS::LOG_STD_LOWER_BOUND, (T)SPEC::PARAMETERS::LOG_STD_UPPER_BOUND);
                T action_sample = get(layer.pre_squashing, 0, action_i);
                T one_minus_action_square_plus_eps = (1-action*action + SPEC::PARAMETERS::LOG_PROBABILITY_EPSILON);
                T action_log_prob = random::normal_distribution::log_prob(device.random, mu, log_std_clamped, action_sample) - math::log(typename DEVICE::SPEC::MATH{}, one_minus_action_square_plus_eps);
                entropy += -action_log_prob;
            }
            add_scalar(device, device.logger, "actor_entropy", entropy, 1000);
        }
        for(TI batch_i = 0; batch_i < INTERNAL_BATCH_SIZE; batch_i++){
            backward_full_per_sample(device, layer, input, d_output, d_input, buffer, alpha, batch_i, mode);
        }
        T d_log_alpha = sum(device, buffer.d_log_alpha);
        add_scalar(device, device.logger, "actor_alpha", alpha, 1000);

        // TODO: change INTERNAL_BATCH_SIZE to sum(reset) if MASK_NON_TERMINAL is used
        if constexpr(mode::is<MODE, nn::layers::gru::ResetMode>){
//            d_log_alpha /= cast_reduce_sum<T>(device, mode.reset_container);
            TI num_resets = nn::layers::gru::mode::num_resets(device, mode);
            d_log_alpha /= num_resets;
        }
        else{
            d_log_alpha /= INTERNAL_BATCH_SIZE;
        }
        increment(device, layer.log_alpha.gradient, d_log_alpha, 0); // note if changing the BATCH_SIZE to INTERNAL_BATCH_SIZE (loss: mean over BATCH & sum over SEQ_LEN vs mean over BATCH & mean over SEQ_LEN) mind to also change it in the sac/operations_generic.h
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto output(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<SPEC>& l){
        // return view(device, l.output);
        auto tensor_flat = to_tensor(device, l.output);
        auto tensor = view_memory<typename SPEC::OUTPUT_SHAPE>(device, tensor_flat);
        return tensor;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

// Tensor proxies
RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools{
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void evaluate_step(DEVICE& device, const nn::layers::sample_and_squash::LayerForward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::sample_and_squash::State& state, Tensor<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        evaluate(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::sample_and_squash::LayerBackward<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, buffer, rng, mode);
        copy(device, device, layer.output, matrix_view_output);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        forward(device, layer, matrix_view_input, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename OUTPUT_SPEC, typename BUFFER_SPEC, typename RNG, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void forward(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<OUTPUT_SPEC>& output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_output = matrix_view(device, output);
        forward(device, layer, matrix_view_input, matrix_view_output, buffer, rng, mode);
    }
    template<typename DEVICE, typename LAYER_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_input(DEVICE& device, const nn::layers::sample_and_squash::LayerBackward<LAYER_SPEC>& layer, const Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}){
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_input(device, layer, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        backward(device, layer, matrix_view_input, matrix_view_d_output, buffer, mode);
    }

    template<typename DEVICE, typename LAYER_SPEC, typename INPUT_SPEC, typename D_OUTPUT_SPEC, typename D_INPUT_SPEC, typename BUFFER_SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT void backward_full(DEVICE& device, nn::layers::sample_and_squash::LayerGradient<LAYER_SPEC>& layer, const Tensor<INPUT_SPEC>& input, Tensor<D_OUTPUT_SPEC>& d_output, Tensor<D_INPUT_SPEC>& d_input, nn::layers::sample_and_squash::Buffer<BUFFER_SPEC>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto matrix_view_input = matrix_view(device, input);
        auto matrix_view_d_output = matrix_view(device, d_output);
        auto matrix_view_d_input = matrix_view(device, d_input);
        backward_full(device, layer, matrix_view_input, matrix_view_d_output, matrix_view_d_input, buffer, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::sample_and_squash::LayerForward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        return false;
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::sample_and_squash::LayerBackward<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::sample_and_squash::LayerForward<SPEC>&>(l), mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.pre_squashing, mode) || is_nan(device, l.noise, mode);
    }
    template <typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    RL_TOOLS_FUNCTION_PLACEMENT bool is_nan(DEVICE& device, const rl_tools::nn::layers::sample_and_squash::LayerGradient<SPEC>& l, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        bool upstream_nan = is_nan(device, static_cast<const rl_tools::nn::layers::sample_and_squash::LayerBackward<SPEC>&>(l), mode);
        upstream_nan =  upstream_nan || is_nan(device, l.log_alpha, mode);
        if constexpr(mode::is<MODE, nn::parameters::mode::ParametersOnly>){
            return upstream_nan;
        }
        return upstream_nan || is_nan(device, l.log_probabilities, mode) || is_nan(device, l.output, mode);
    }
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT auto gradient_norm(DEVICE& device, const nn::layers::sample_and_squash::LayerGradient<SPEC>& layer) {
        return 0;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
#endif
