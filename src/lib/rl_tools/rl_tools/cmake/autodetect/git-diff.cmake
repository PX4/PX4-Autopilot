# ==============================================================================
# Git Diff Tracking for ExTrack
# ==============================================================================
# Captures git state at build time for both the library itself and parent project (if RLtools is e.g. used as a submodule using add_subdirectory).
# For the parent project to be tracked properly it should have the relevant files in ./include and ./src.
# 
# When used standalone: tracks only rl-tools repository
# When used via add_subdirectory: tracks both rl-tools AND parent project
#
# Configuration:
#   RL_TOOLS_ENABLE_GIT_DIFF - Enable/disable git diff tracking (default: OFF)
# ==============================================================================

option(RL_TOOLS_ENABLE_GIT_DIFF "Enable embedding git diff into ExTrack runs" ON)

if(NOT RL_TOOLS_ENABLE_GIT_DIFF)
    return()
endif()

find_package(Git QUIET)
if(NOT GIT_FOUND)
    message(STATUS "Git not found - git diff tracking disabled")
    return()
endif()

get_filename_component(RL_TOOLS_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
file(REAL_PATH "${RL_TOOLS_ROOT_DIR}" RL_TOOLS_ROOT_DIR_NORMALIZED)
file(REAL_PATH "${CMAKE_SOURCE_DIR}" CMAKE_SOURCE_DIR_NORMALIZED)
if(NOT CMAKE_SOURCE_DIR_NORMALIZED STREQUAL RL_TOOLS_ROOT_DIR_NORMALIZED)
    set(RL_TOOLS_PARENT_DIR "${CMAKE_SOURCE_DIR_NORMALIZED}")
    set(RL_TOOLS_HAS_PARENT TRUE)
else()
    set(RL_TOOLS_HAS_PARENT FALSE)
endif()

set(RL_TOOLS_ROOT_DIR "${RL_TOOLS_ROOT_DIR_NORMALIZED}")

set(RL_TOOLS_IS_GIT_REPO FALSE)
if(EXISTS "${RL_TOOLS_ROOT_DIR}/.git")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --git-dir
        WORKING_DIRECTORY ${RL_TOOLS_ROOT_DIR}
        RESULT_VARIABLE GIT_CHECK_RESULT
        OUTPUT_QUIET ERROR_QUIET
    )
    if(GIT_CHECK_RESULT EQUAL 0)
        set(RL_TOOLS_IS_GIT_REPO TRUE)
    endif()
endif()

set(RL_TOOLS_PARENT_IS_GIT_REPO FALSE)
if(RL_TOOLS_HAS_PARENT AND EXISTS "${RL_TOOLS_PARENT_DIR}/.git")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --git-dir
        WORKING_DIRECTORY ${RL_TOOLS_PARENT_DIR}
        RESULT_VARIABLE GIT_CHECK_RESULT
        OUTPUT_QUIET ERROR_QUIET
    )
    if(GIT_CHECK_RESULT EQUAL 0)
        set(RL_TOOLS_PARENT_IS_GIT_REPO TRUE)
    endif()
endif()

if(NOT RL_TOOLS_IS_GIT_REPO AND NOT RL_TOOLS_PARENT_IS_GIT_REPO)
    message(STATUS "No git repositories detected - git diff tracking disabled")
    return()
endif()

message(STATUS "Git diff tracking enabled:")
message(STATUS "  Library: ${RL_TOOLS_ROOT_DIR} [git: ${RL_TOOLS_IS_GIT_REPO}]")
if(RL_TOOLS_HAS_PARENT)
    message(STATUS "  Project: ${RL_TOOLS_PARENT_DIR} [git: ${RL_TOOLS_PARENT_IS_GIT_REPO}]")
endif()

set(GIT_DIFF_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/rl_tools/extrack/git_diff.cpp")
set(GIT_DIFF_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/rl_tools_generate_git_diff.cmake")

file(WRITE "${GIT_DIFF_SCRIPT}" "\
# Auto-generated git diff capture script
# This runs at build time to capture current git state

set(GIT_EXECUTABLE \"${GIT_EXECUTABLE}\")
set(OUTPUT_FILE \"${GIT_DIFF_OUTPUT}\")
set(RL_TOOLS_ROOT \"${RL_TOOLS_ROOT_DIR}\")
set(RL_TOOLS_IS_REPO ${RL_TOOLS_IS_GIT_REPO})
set(RL_TOOLS_PARENT_IS_GIT_REPO ${RL_TOOLS_PARENT_IS_GIT_REPO})
set(PARENT_ROOT \"${RL_TOOLS_PARENT_DIR}\")
set(PARENT_IS_REPO ${RL_TOOLS_PARENT_IS_GIT_REPO})

# Helper function to safely execute git commands
function(git_capture VAR REPO_DIR)
    execute_process(
        COMMAND \${GIT_EXECUTABLE} \${ARGN}
        WORKING_DIRECTORY \${REPO_DIR}
        OUTPUT_VARIABLE OUTPUT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(\${VAR} \"\${OUTPUT}\" PARENT_SCOPE)
endfunction()

# Namespace structure for dual repository tracking
set(CONTENT \"namespace rl_tools { namespace utils { namespace extrack { namespace git {\\n\")

# ============================================================================
# Library Repository (rl-tools itself)
# ============================================================================
set(CONTENT \"\${CONTENT}namespace rl_tools {\\n\")

if(RL_TOOLS_IS_REPO)
    git_capture(COMMIT \${RL_TOOLS_ROOT} rev-parse HEAD)
    git_capture(DIFF \${RL_TOOLS_ROOT} diff)
    git_capture(DIFF_COLOR \${RL_TOOLS_ROOT} diff --color=always)
    git_capture(WORD_DIFF \${RL_TOOLS_ROOT} diff --word-diff)
    git_capture(WORD_DIFF_COLOR \${RL_TOOLS_ROOT} diff --word-diff --color=always)
    git_capture(DIFF_STAGED \${RL_TOOLS_ROOT} diff --cached)
    git_capture(DIFF_STAGED_COLOR \${RL_TOOLS_ROOT} diff --cached --color=always)
    git_capture(WORD_DIFF_STAGED \${RL_TOOLS_ROOT} diff --cached --word-diff)
    git_capture(WORD_DIFF_STAGED_COLOR \${RL_TOOLS_ROOT} diff --cached --word-diff --color=always)
    
    if(NOT COMMIT)
        set(COMMIT \"unknown\")
    endif()
else()
    set(COMMIT \"not_a_repository\")
    set(DIFF \"\")
    set(DIFF_COLOR \"\")
    set(WORD_DIFF \"\")
    set(WORD_DIFF_COLOR \"\")
    set(DIFF_STAGED \"\")
    set(DIFF_STAGED_COLOR \"\")
    set(WORD_DIFF_STAGED \"\")
    set(WORD_DIFF_STAGED_COLOR \"\")
endif()

set(CONTENT \"\${CONTENT}    extern const char* const commit = R\\\"rl_tools_git(\${COMMIT})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff = R\\\"rl_tools_git(\${DIFF})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_color = R\\\"rl_tools_git(\${DIFF_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff = R\\\"rl_tools_git(\${WORD_DIFF})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_color = R\\\"rl_tools_git(\${WORD_DIFF_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_staged = R\\\"rl_tools_git(\${DIFF_STAGED})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_staged_color = R\\\"rl_tools_git(\${DIFF_STAGED_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_staged = R\\\"rl_tools_git(\${WORD_DIFF_STAGED})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_staged_color = R\\\"rl_tools_git(\${WORD_DIFF_STAGED_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}} // namespace rl_tools\\n\\n\")

# ============================================================================
# Parent Project Repository
# ============================================================================
set(CONTENT \"\${CONTENT}namespace project {\\n\")

# Boolean flag indicating if a parent project exists
if(RL_TOOLS_PARENT_IS_GIT_REPO)
    set(CONTENT \"\${CONTENT}    extern const bool available = true;\\n\")
else()
    set(CONTENT \"\${CONTENT}    extern const bool available = false;\\n\")
endif()

if(PARENT_IS_REPO)
    git_capture(COMMIT \${PARENT_ROOT} rev-parse HEAD)
    git_capture(DIFF \${PARENT_ROOT} diff)
    git_capture(DIFF_COLOR \${PARENT_ROOT} diff --color=always)
    git_capture(WORD_DIFF \${PARENT_ROOT} diff --word-diff)
    git_capture(WORD_DIFF_COLOR \${PARENT_ROOT} diff --word-diff --color=always)
    git_capture(DIFF_STAGED \${PARENT_ROOT} diff --cached)
    git_capture(DIFF_STAGED_COLOR \${PARENT_ROOT} diff --cached --color=always)
    git_capture(WORD_DIFF_STAGED \${PARENT_ROOT} diff --cached --word-diff)
    git_capture(WORD_DIFF_STAGED_COLOR \${PARENT_ROOT} diff --cached --word-diff --color=always)
    
    if(NOT COMMIT)
        set(COMMIT \"unknown\")
    endif()
else()
    set(COMMIT \"not_a_repository\")
    set(DIFF \"\")
    set(DIFF_COLOR \"\")
    set(WORD_DIFF \"\")
    set(WORD_DIFF_COLOR \"\")
    set(DIFF_STAGED \"\")
    set(DIFF_STAGED_COLOR \"\")
    set(WORD_DIFF_STAGED \"\")
    set(WORD_DIFF_STAGED_COLOR \"\")
endif()

set(CONTENT \"\${CONTENT}    extern const char* const commit = R\\\"rl_tools_git(\${COMMIT})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff = R\\\"rl_tools_git(\${DIFF})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_color = R\\\"rl_tools_git(\${DIFF_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff = R\\\"rl_tools_git(\${WORD_DIFF})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_color = R\\\"rl_tools_git(\${WORD_DIFF_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_staged = R\\\"rl_tools_git(\${DIFF_STAGED})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const diff_staged_color = R\\\"rl_tools_git(\${DIFF_STAGED_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_staged = R\\\"rl_tools_git(\${WORD_DIFF_STAGED})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}    extern const char* const word_diff_staged_color = R\\\"rl_tools_git(\${WORD_DIFF_STAGED_COLOR})rl_tools_git\\\";\\n\")
set(CONTENT \"\${CONTENT}} // namespace project\\n\\n\")

set(CONTENT \"\${CONTENT}}}}}\") # close namespaces: git, extrack, utils, rl_tools

# Write output
file(MAKE_DIRECTORY \"${CMAKE_CURRENT_BINARY_DIR}/rl_tools/extrack\")
file(WRITE \"\${OUTPUT_FILE}\" \"\${CONTENT}\")
")

set(GIT_DIFF_DEPENDENCIES "")
if(RL_TOOLS_IS_GIT_REPO)
    file(GLOB_RECURSE RL_TOOLS_SOURCES
        CONFIGURE_DEPENDS
        "${RL_TOOLS_ROOT_DIR}/src/*"
        "${RL_TOOLS_ROOT_DIR}/include/*"
    )
    list(APPEND GIT_DIFF_DEPENDENCIES ${RL_TOOLS_SOURCES})
endif()

if(RL_TOOLS_PARENT_IS_GIT_REPO)
    file(GLOB_RECURSE PARENT_SOURCES
        CONFIGURE_DEPENDS
        "${RL_TOOLS_PARENT_DIR}/src/*"
        "${RL_TOOLS_PARENT_DIR}/include/*"
    )
    list(APPEND GIT_DIFF_DEPENDENCIES ${PARENT_SOURCES})
endif()

add_custom_command(
    OUTPUT ${GIT_DIFF_OUTPUT}
    COMMAND ${CMAKE_COMMAND} -P "${GIT_DIFF_SCRIPT}"
    DEPENDS ${GIT_DIFF_DEPENDENCIES}
    COMMENT "Capturing git state for ExTrack"
    VERBATIM
)

add_library(rl_tools_git_diff STATIC ${GIT_DIFF_OUTPUT})
target_compile_features(rl_tools_git_diff PRIVATE cxx_std_11)
target_compile_definitions(rl_tools_git_diff PUBLIC RL_TOOLS_EXTRACK_GIT_DIFF)

if(TARGET rl_tools_full)
    target_link_libraries(rl_tools_full INTERFACE rl_tools_git_diff)
endif()
