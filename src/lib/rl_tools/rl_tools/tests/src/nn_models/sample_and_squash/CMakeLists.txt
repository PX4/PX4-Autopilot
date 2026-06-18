add_executable(
        test_nn_models_sample_and_squash
        sample_and_squash.cpp
)

target_link_libraries(
        test_nn_models_sample_and_squash
        RLtools::Core
)