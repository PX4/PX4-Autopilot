# Inject the same diagnostic option into both revisions without changing their
# optimisation flags or requiring the baseline to contain an analysis option.
# Usage: -DCMAKE_PROJECT_INCLUDE=/absolute/path/to/memory_analysis.cmake
if(NOT CMAKE_C_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	message(FATAL_ERROR "Memory frame analysis currently requires GCC")
endif()

add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-fstack-usage>)
