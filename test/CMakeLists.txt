px4_add_git_submodule(TARGET git_fuzztest PATH "fuzztest")

message(STATUS "Adding fuzztest")
# This will also add GTest
add_subdirectory(fuzztest EXCLUDE_FROM_ALL)

# Ensure there's no -R without any filter expression since that trips newer ctest versions
if(TESTFILTER)
    set(TESTFILTERARG "-R")
else()
    set(TESTFILTERARG "")
endif()

add_custom_target(test_results
        # antlr4_tests_NOT_BUILT gets added by fuzztest
        COMMAND GTEST_COLOR=1 ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test ${TESTFILTERARG} ${TESTFILTER} --exclude-regex "antlr4_tests_NOT_BUILT"
        DEPENDS
        px4
        examples__dyn_hello
        USES_TERMINAL
        COMMENT "Running tests"
        WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)
