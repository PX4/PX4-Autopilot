cmake_minimum_required(VERSION 3.22)

project(libmodal_json)

set(LIBNAME modal_json)

# Strawson's list of standard list of default gcc flags. Yes, I treat
# warnings as errors. Warnings exist to point out sloppy code and potential
# failure points for good reason. We do not approve of sloppy code.
set(CMAKE_C_FLAGS "-std=gnu99 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=gnu++11 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

# tell the linker not to worry about missing symbols in libraries
# core Modal AI libraries often have a lot of dependencies and this prevents
# higher level projects from needing to install so many dependencies at compile time
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")



# add targets from subdirectories
add_subdirectory(library)

# optionally build example binaries
set(BUILD_EXAMPLES ON CACHE BOOL "Whether to build examples or not")
if(BUILD_EXAMPLES)
	message(STATUS "Building examples")
	add_subdirectory(examples)
else()
	message(STATUS "Skipping examples")
endif()

