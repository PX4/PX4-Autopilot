cmake_minimum_required(VERSION 2.8)

project(px4-firmware C CXX ASM)

message(STATUS "CMAKE CURRENTLY CAN ONLY BUILD THE NUTTX ARCHIVES:"
   " (target) link_exports")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(CMakeParseArguments)

##############################################################################
# project parameters
##############################################################################

# options
#=============================================================================
option(FIRMWARE_BUILD "attempt firmware build, not yet working" OFF)
set(NUTTX_BUILD_THREADS "1" CACHE STRING
    "number of threads to use when building NuttX")

# package config
#=============================================================================

# packaging
set(CPACK_PACKAGE_CONTACT "px4users@googlegroups.com")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")

# build config
#=============================================================================

# boards
set(BOARDS
    aerocore
    px4fmu-v1
    px4fmu-v2
    px4io-v1
    px4io-v2
    )

# defines how to build all modules
# locted in cmake dir as .cmake module
set(MODULES_FILE px4_modules)

# defines how to build each board
# locted in cmake dir as .cmake module
set(BUILD_CONFIGURATIONS
    px4fmu-v2_default
    sim_default
    )

# build flags
#=============================================================================
set(CORTEXM4_CPU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -march=armv7e-m
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    )
set(OPT_FLAGS
    -O3
    -g
    -fno-strict-aliasing
    -fno-strength-reduce
    -fomit-frame-pointer
    -funsafe-math-optimizations
    -fno-builtin-printf
    -ffunction-sections
    -fdata-sections
    )
set(C_FLAGS
    -std=gnu99
    -fno-common
    )
set(CXX_FLAGS
    -fno-exceptions
    -fno-rtti
    -std=gnu++0x
    -fno-threadsafe-statics
    )
set(WARNINGS
    -Wall
    -Wextra
    -Wdouble-promotion
    -Wshadow
    -Wfloat-equal
    -Wframe-larger-than=1024
    -Wpointer-arith
    -Wlogical-op
    -Wmissing-declarations
    -Wpacked
    -Wno-unused-parameter
    -Werror=format-security
    -Werror=array-bounds
    -Wfatal-errors
    -Wformat=1
    -Werror=unused-but-set-variable
    -Werror=unused-variable
    -Werror=double-promotion
    -Werror=reorder
    #-Wcast-qual  - generates spurious noreturn attribute warnings,
    #               try again later
    #-Wconversion - would be nice, but too many "risky-but-safe"
    #               conversions in the code
    #-Wcast-align - would help catch bad casts in some cases,
    #               but generates too many false positives
    )
set(C_WARNINGS
    -Wbad-function-cast
    -Wstrict-prototypes
    -Wold-style-declaration
    -Wmissing-parameter-type
    -Wmissing-prototypes
    -Wnested-externs
    )
set(CXX_WARNINGS
    -Wno-missing-field-initializers
    )
set(LD_FLAGS
    -Wl,--warn-common
    -Wl,--gc-sections
    )

# git submodules
#=============================================================================
set(GIT_SUBMODULES
    NuttX
    mavlink/include/mavlink/v1.0
    uavcan
    unittests/gtest
    )
set(NuttX_path "NuttX")
set(mavlink_include_mavlink_v1.0_path "mavlink/include/mavlink/v1.0")
set(uavcan_path "src/lib/uavcan")
set(unittests_gtest_path "unittests/gtest")

# dependencies/ programs/ testing/ packaging
#=============================================================================
find_program(OPENOCD openocd HINT ../../sat/bin/)
enable_testing()
include(CPack)

##############################################################################
# Function/Macros
##############################################################################
function(join VARNAME)
    string (REGEX REPLACE "([^\\]|^);" "\\1 " _TMP_STR "${${VARNAME}}")
    #now fixes escaping
    string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}")
    set(${VARNAME} "${_TMP_STR}" PARENT_SCOPE)
endfunction()

macro(generate_firmware NAME)
    add_custom_target(firmware_${NAME} ALL DEPENDS ${NAME}.px4)
    add_custom_command(OUTPUT ${NAME}.px4
        COMMAND objcopy --output-format=binary ${NAME} ${NAME}.bin
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py
            --board_id 6 > ${NAME}_prototype.px4
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py
            --prototype ${NAME}_prototype.px4 --image ${NAME}.bin > ${NAME}.px4
        DEPENDS ${NAME})
endmacro()

macro(add_nuttx_export BOARD)
    set(BOARD_NUTTX_SRC ${CMAKE_BINARY_DIR}/${BOARD}/NuttX)

    # copy
    add_custom_command(OUTPUT nuttx_copy_${BOARD}.stamp
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/${BOARD}
        COMMAND cp -r ${CMAKE_BINARY_DIR}/NuttX ${BOARD_NUTTX_SRC}
        COMMAND touch nuttx_copy_${BOARD}.stamp)
    add_custom_target(nuttx_copy_${BOARD}
        DEPENDS nuttx_copy_${BOARD}.stamp nuttx_patch)

    # export
    add_custom_command(OUTPUT ${BOARD}.export
        COMMAND echo Configuring NuttX for ${BOARD}
        COMMAND make -C${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS}
            -r --quiet distclean
        COMMAND cp -r ${CMAKE_SOURCE_DIR}/nuttx-configs/${BOARD}
            ${BOARD_NUTTX_SRC}/nuttx/configs
        COMMAND cd ${BOARD_NUTTX_SRC}/nuttx/tools &&
            ./configure.sh ${BOARD}/nsh
        COMMAND echo Exporting NuttX for ${BOARD}
        COMMAND make -C ${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS}
            -r CONFIG_ARCH_BOARD=${BOARD} export
        COMMAND cp -r ${BOARD_NUTTX_SRC}/nuttx/nuttx-export.zip
            ${BOARD}.export
        DEPENDS nuttx_copy_${BOARD})

    # extract
    add_custom_command(OUTPUT nuttx_export_${BOARD}.stamp
        COMMAND rm -rf ${BOARD_NUTTX_SRC}/nuttx-export
        COMMAND unzip ${BOARD}.export -d ${BOARD_NUTTX_SRC}
        COMMAND touch nuttx_export_${BOARD}.stamp
        DEPENDS ${BOARD}.export)
    add_custom_target(NuttX_${BOARD}
        DEPENDS nuttx_export_${BOARD}.stamp)

    # this symbolic linking is to allow cmake to build all of the
    # archives for the current makefile, it won't be needed in a
    # complete cmake build
    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/Archives
        COMMAND rm -f ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        COMMAND ln -sf ${CMAKE_BINARY_DIR}/${BOARD}.export
            ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        DEPENDS ${BOARD}.export)
    add_custom_target(link_export_${BOARD}
        DEPENDS ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export)
endmacro()

function(prepend_to_list NEW_LIST LIST STR)
    set(${NEW_LIST})
    foreach(SRC IN LISTS ${LIST})
        list(APPEND ${NEW_LIST} "${STR}/${SRC}")
    endforeach()
    set(${NEW_LIST} ${${NEW_LIST}} PARENT_SCOPE)
endfunction()

function(add_module)
    cmake_parse_arguments(ARG
        "" "NAME;PATH;STACK;MAIN" "SRCS;HDRS" "" ${ARGN})
    prepend_to_list(SRCS_PATH ARG_SRCS ${ARG_PATH})
    prepend_to_list(HDRS_PATH ARG_HDRS ${ARG_PATH})
    set(${ARG_NAME}_NAME ${ARG_NAME} PARENT_SCOPE)
    set(${ARG_NAME}_PATH ${ARG_PATH} PARENT_SCOPE)
    set(${ARG_NAME}_SRCS ${SRCS_PATH} PARENT_SCOPE)
    set(${ARG_NAME}_HDRS ${HDRS_PATH} PARENT_SCOPE)
    set(${ARG_NAME}_STACK ${ARG_STACK} PARENT_SCOPE)
    set(${ARG_NAME}_MAIN ${ARG_MAIN} PARENT_SCOPE)
    message(STATUS "added module")
    message(STATUS "\tname:\t${ARG_NAME}")
    message(STATUS "\tpath:\t${ARG_PATH}")
    message(STATUS "\tsrcs:\t${ARG_SRCS}")
    #message(STATUS "\tsrcs:\t${SRCS_PATH}")
    message(STATUS "\thdrs:\t${ARG_HDRS}")
    #message(STATUS "\thdrs:\t${HDRS_PATH}")
    message(STATUS "\tstack:\t${ARG_STACK}")
    message(STATUS "\tmain:\t${ARG_MAIN}")
endfunction(add_module)

##############################################################################
# build setup
##############################################################################
message(STATUS "build: ${CMAKE_SYSTEM_NAME}")

if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
    set(SIM False)
else()
    message(STATUS "In order to build firmware for board, remove all files"
       " in the build directory and call: "
       "cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-arm-none-eabi.cmake ..")
    set(SIM True)
    set(BOARDS sim)
endif()

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/.git)
    set(GIT_REPO True)
else()
    set(GIT_REPO False)
endif()

if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
    set(CPU_FLAGS ${CORTEXM4_CPU_FLAGS})
else()
    set(CPU_FLAGS)
endif()
set(CMAKE_C_FLAGS
    ${CPU_FLAGS}
    ${C_FLAGS}
    ${WARNINGS}
    ${C_WARNINGS}
    ${OPT_FLAGS}
    )
join(CMAKE_C_FLAGS)
set(CMAKE_CXX_FLAGS
    ${CPU_FLAGS}
    ${CXX_FLAGS}
    ${WARNINGS}
    ${CXX_WARNINGS}
    ${OPT_FLAGS}
    )
join(CMAKE_CXX_FLAGS)
set(CMAKE_EXE_LINKER_FLAGS ${LD_FLAGS})
join(CMAKE_EXE_LINKER_FLAGS)
message(STATUS "cflags: ${CMAKE_C_FLAGS}")


##############################################################################
# Git Submodules
##############################################################################

set(GIT_MOD_DIR ${CMAKE_SOURCE_DIR}/.git/modules)

# convenience target to nuke all submodules
add_custom_target(submodule_clean
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND git submodule deinit -f .
    COMMAND rm -rf .git/modules/*
    )

# add git submodule targets
foreach(MOD IN LISTS GIT_SUBMODULES)
    string(REPLACE "/" "_" MOD_NAME ${MOD})
    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/git_${MOD_NAME}.stamp
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND git submodule init ${${MOD}_path}
        COMMAND git submodule update ${${MOD}_path}
        COMMAND touch ${CMAKE_BINARY_DIR}/git_${MOD_NAME}.stamp
        )
    add_custom_target(git_${MOD_NAME}
        DEPENDS git_${MOD_NAME}.stamp
        )
endforeach()

# clone
add_custom_command(OUTPUT nuttx_clone.stamp
    COMMAND git clone ${CMAKE_SOURCE_DIR}/.git/modules/NuttX
    COMMAND touch nuttx_clone.stamp)
add_custom_target(nuttx_clone DEPENDS nuttx_clone.stamp
    DEPENDS git_NuttX)

##############################################################################
# NuttX
##############################################################################

# patch
add_custom_target(nuttx_patch)
file(GLOB NUTTX_PATCHES RELATIVE ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/nuttx-patches/*.patch)
foreach(PATCH IN LISTS NUTTX_PATCHES)
    string(REPLACE "/" "_" PATCH_NAME ${PATCH})
    message(STATUS nuttx-patch: ${PATCH})
    add_custom_command(OUTPUT nuttx_patch_${PATCH_NAME}.stamp
        COMMAND patch -p0 -N  < ${CMAKE_SOURCE_DIR}/${PATCH}
        COMMAND touch nuttx_patch_${PATCH_NAME}.stamp
        )
    add_custom_target(nuttx_patch_${PATCH_NAME}
        DEPENDS nuttx_patch_${PATCH_NAME}.stamp nuttx_clone)
    add_dependencies(nuttx_patch nuttx_patch_${PATCH_NAME})
endforeach()
add_dependencies(nuttx_patch nuttx_clone)

# keep track of all exports to link
add_custom_target(link_exports)

# common include directories to all builds
include_directories(
    src
    src/include
    src/modules)

# NuttX
foreach(BOARD IN LISTS BOARDS)
    add_nuttx_export(${BOARD})
    add_dependencies(link_exports link_export_${BOARD})
endforeach(BOARD IN LISTS BOARDS)

##############################################################################
# Firmware
##############################################################################

if (FIRMWARE_BUILD)

    # initialize lists of configurations to empty
    foreach(BOARD IN LISTS ${BOARD})
        set(${BOARD}_configs)
    endforeach()

    # module definitions
    include(${MODULES_FILE})

    # configurations
    foreach(CONFIG IN LISTS BUILD_CONFIGURATIONS)
        include(${CONFIG})
    endforeach()

    foreach(BOARD IN LISTS BOARDS)
        message(STATUS ${BOARD})
        set(EXPORT_DIR ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export/)
        foreach(CONFIG IN LISTS ${BOARD}_configs)
            message(STATUS \t${CONFIG})

            # board includes
            include_directories(
                ${EXPORT_DIR}
                ${EXPORT_DIR}/include
                ${EXPORT_DIR}/arch/chip
                )

            # build all modules
            set(MODULES)
            foreach(MODULE IN LISTS ${BOARD}_${CONFIG}_modules)
                message(STATUS "\t\t${MODULE}")
                message(STATUS "\t\t\tname:\t${${MODULE}_NAME}")
                message(STATUS "\t\t\tstack:\t${${MODULE}_STACK}")
                message(STATUS "\t\t\tmain:\t${${MODULE}_MAIN}")
                message(STATUS "\t\t\tsrcs:\t${${MODULE}_SRCS}")
                # name for module
                # define how a module is built
                set(MOD ${MODULE}_${BOARD}_${CONFIG})
                add_library(${MOD} ${${MODULE}_SRCS})
                # set NuttX for board as dep
                add_dependencies(${MOD} NuttX_${BOARD})
                # add module to list of modules for board config
                list(APPEND MODULES ${MOD})
            endforeach()

            # startup library
            file(GLOB STARTUP_OBJS ${EXPORT_DIR}/startup/*.o)
            if(STARTUP_OBJS)
                message(STATUS startup objects: ${STARTUP_OBJS})
                add_library(startup STATIC
                    ${STARTUP_OBJS})
                set_source_files_properties(${STARTUP_OBJS}
                    PROPERTIES
                    # Identifies this as an object file
                    EXTERNAL_OBJECT TRUE
                    # Avoids need for file to exist at configure-time
                    GENERATED TRUE
                    )
                set_target_properties(startup PROPERTIES LINKER_LANGUAGE C)
            else()
                set(startup)
            endif()

            # build the executable
            link_directories(
                ${EXPORT_DIR}/libs)
            link_libraries(
                startup
                apps
                ${MODULES}
                nuttx
                startup
                apps
                )
            add_executable(${BOARD}_${CONFIG} cmake/builtin_commands.c)

            # generate the firmware
            if (NOT SIM)
                set_target_properties(${BOARD}_${CONFIG} PROPERTIES
                    LINK_FLAGS "-T${EXPORT_DIR}/build/ld.script"
                    #COMPILE_FLAGS "-Werror"
                )
                generate_firmware(${BOARD}_${CONFIG})
            else()
                target_link_libraries(${BOARD}_${CONFIG} pthread z)
            endif()

        endforeach()
    endforeach(BOARD IN LISTS BOARDS)

endif(FIRMWARE_BUILD)

##############################################################################
# Testing
##############################################################################
add_test(fake echo "hi")


##############################################################################
# vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 ft=cmake:
