cmake_minimum_required(VERSION 3.10)

add_executable(lockstep_scheduler_test
    src/lockstep_scheduler_test.cpp
)

target_link_libraries(lockstep_scheduler_test
    lockstep_scheduler
)

target_compile_options(lockstep_scheduler_test PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:MSVC>:/WX>
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<NOT:$<CONFIG:Debug>>>:/O2>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wextra>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-O2>
)
