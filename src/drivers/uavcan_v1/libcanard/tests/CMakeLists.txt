# This software is distributed under the terms of the MIT License.
# Copyright (c) 2016-2020 UAVCAN Development Team.

cmake_minimum_required(VERSION 3.12)
project(canard_tests C CXX)
enable_testing()

if (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    # assert() shall be disabled in release builds to enable testing of bad free() calls.
    add_definitions(-DNDEBUG=1)
endif ()

set(library_dir "${CMAKE_SOURCE_DIR}/../libcanard")

# Use -DNO_STATIC_ANALYSIS=1 to suppress static analysis.
# If not suppressed, the tools used here shall be available, otherwise the build will fail.
if (NOT NO_STATIC_ANALYSIS)
    # clang-tidy (separate config files per directory)
    find_program(clang_tidy NAMES clang-tidy-12 clang-tidy-11 clang-tidy-10 clang-tidy)
    if (NOT clang_tidy)
        message(FATAL_ERROR "Could not locate clang-tidy")
    endif ()
    message(STATUS "Using clang-tidy: ${clang_tidy}")
    set(CMAKE_C_CLANG_TIDY   ${clang_tidy})
    set(CMAKE_CXX_CLANG_TIDY ${clang_tidy})

    # clang-format
    find_program(clang_format NAMES clang-format-12 clang-format-11 clang-format-10 clang-format)
    if (NOT clang_format)
        message(FATAL_ERROR "Could not locate clang-format")
    endif ()
    file(GLOB format_files
         ${library_dir}/*.[ch]
         ${CMAKE_SOURCE_DIR}/*.[ch]pp)
    message(STATUS "Using clang-format: ${clang_format}; files: ${format_files}")
    add_custom_target(format COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${format_files})
endif ()

# C options
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -pedantic -fstrict-aliasing")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wdouble-promotion -Wswitch-enum -Wfloat-equal -Wundef")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wconversion -Wtype-limits")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")

# C++ options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic -fstrict-aliasing")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wswitch-enum -Wfloat-equal -Wundef")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-promo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")

include_directories(catch ${library_dir})
add_definitions(-DCATCH_CONFIG_FAST_COMPILE=1)

set(common_sources catch/main.cpp ${library_dir}/canard.c ${library_dir}/canard_dsdl.c)

function(gen_test name files compile_definitions compile_flags link_flags c_standard)
    add_executable(${name} ${common_sources} ${files})
    target_compile_definitions(${name} PUBLIC ${compile_definitions})
    target_link_libraries(${name} pthread)
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${compile_flags}" LINK_FLAGS "${link_flags}" C_STANDARD "${c_standard}")
    add_test("run_${name}" "${name}" --rng-seed time)
endfunction()

function(gen_test_matrix name files compile_definitions)
    gen_test("${name}_x64_c99" "${files}" "${compile_definitions}" "-m64" "-m64" "99")
    gen_test("${name}_x32_c99" "${files}" "${compile_definitions}" "-m32" "-m32" "99")
    gen_test("${name}_x64_c11" "${files}" "${compile_definitions}" "-m64" "-m64" "11")
    gen_test("${name}_x32_c11" "${files}" "${compile_definitions}" "-m32" "-m32" "11")
    # Coverage is only available for GCC builds.
    if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        gen_test("${name}_cov" "${files}" "${compile_definitions}" "-g -O0 --coverage" "--coverage" "11")
    endif ()
endfunction()

gen_test_matrix(test_private
                "test_private_crc.cpp;test_private_rx.cpp;test_private_tx.cpp;test_dsdl.cpp"
                CANARD_CONFIG_EXPOSE_PRIVATE=1)

# We assume here that the target platform is little-endian. If it's not, the test will fail.
# Perhaps this needs fixing: these tests need not be run if the target is not little-endian.
gen_test_matrix(test_private_le
                "test_private_crc.cpp;test_private_rx.cpp;test_private_tx.cpp;test_dsdl.cpp"
                "CANARD_CONFIG_EXPOSE_PRIVATE=1;CANARD_DSDL_CONFIG_LITTLE_ENDIAN=1")

gen_test_matrix(test_public
                "test_public_tx.cpp;test_public_rx.cpp;test_public_roundtrip.cpp;test_self.cpp"
                "")
