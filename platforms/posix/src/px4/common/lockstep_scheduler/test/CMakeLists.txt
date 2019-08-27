cmake_minimum_required(VERSION 2.8.12)

add_executable(lockstep_scheduler_test
    src/lockstep_scheduler_test.cpp
)

target_link_libraries(lockstep_scheduler_test
    lockstep_scheduler
)

target_compile_options(lockstep_scheduler_test PRIVATE -Wall -Wextra -Werror -O2)
