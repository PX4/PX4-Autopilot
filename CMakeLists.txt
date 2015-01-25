cmake_minimum_required(VERSION 2.8)

# toolchain
#-----------------------------------------------------
option(HOST_TEST "do host based testing (no cross-compiling)" OFF)
if (NOT HOST_TEST)
    set(CMAKE_TOOLCHAIN_FILE cmake/Toolchain-arm-none-eabi.cmake)
    message(STATUS "CROSS COMPILING FOR ARM")
endif()

project(px4-firmware C CXX ASM)

set(NUTTX_BUILD_THREADS "1" CACHE STRING "number of threads to use when building NuttX")
option(FIRMWARE_BUILD "attempt firmware build, not yet working" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

message(STATUS "build: ${CMAKE_SYSTEM_NAME}")

# config
# it isn't good repeating all this config from the makefiles
# we need to find a better solution if we want to keep
# both builds becasue this will be a maintenance nightmare
#-----------------------------------------------------

if(HOST_TEST)
    set(BOARDS
        sim
        )
else()
    set(BOARDS
        aerocore
        px4fmu-v1
        px4fmu-v2
        px4io-v1
        px4io-v2
        )
endif()

# initialize lists of configurations to empty
foreach(BOARD IN LISTS ${BOARD})
    set(${BOARD}_configs)
endforeach()

# module definitions
include(px4_modules)

# configurations
include(px4fmu-v2_default)
include(sim_default)

# submodule config info
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

# dependencies/ programs
#-----------------------------------------------------
find_program(OPENOCD openocd HINT ../../sat/bin/)

# testing
#-----------------------------------------------------
enable_testing()

# packaging
#-----------------------------------------------------
set(CPACK_PACKAGE_CONTACT "px4users@googlegroups.com")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
include(CPack)

# functions
#-----------------------------------------------------
function(join VARNAME)
    string (REGEX REPLACE "([^\\]|^);" "\\1 " _TMP_STR "${${VARNAME}}")
    string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
    set(${VARNAME} "${_TMP_STR}" PARENT_SCOPE)
endfunction()

macro(generate_firmware NAME)
    add_custom_target(firmware_${NAME} ALL DEPENDS ${NAME}.px4)
    add_custom_command(OUTPUT ${NAME}.px4
        COMMAND objcopy --output-format=binary ${NAME} ${NAME}.bin
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py --board_id 6 > ${NAME}_prototype.px4
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py --prototype ${NAME}_prototype.px4 --image ${NAME}.bin > ${NAME}.px4
        DEPENDS ${NAME})
endmacro()

macro(add_nuttx_export BOARD)
    set(BOARD_NUTTX_SRC ${CMAKE_BINARY_DIR}/NuttX-${BOARD})
    # create
    add_custom_command(OUTPUT ${BOARD}.export
        COMMAND cp -r ${NUTTX_SRC} ${CMAKE_BINARY_DIR}/NuttX-${BOARD}
        COMMAND echo Configuring NuttX for ${BOARD}
        COMMAND rm -rf ${BOARD_NUTTX_SRC}/nuttx-export
        COMMAND make -C${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS} -r --quiet distclean
        COMMAND cp -r ${CMAKE_SOURCE_DIR}/nuttx-configs/${BOARD} ${BOARD_NUTTX_SRC}/nuttx/configs
        COMMAND cd ${BOARD_NUTTX_SRC}/nuttx/tools && ./configure.sh ${BOARD}/nsh
        COMMAND echo Exporting NuttX for ${BOARD}
        COMMAND make -C ${BOARD_NUTTX_SRC}/nuttx -j${NUTTX_BUILD_THREADS} -r CONFIG_ARCH_BOARD=${BOARD} export
        COMMAND cp -r ${BOARD_NUTTX_SRC}/nuttx/nuttx-export.zip ${BOARD}.export)

    # extract
    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/NuttX-${BOARD}/nuttx-export/.stamp
        COMMAND rm -rf ${CMAKE_BINARY_DIR}/NuttX-${BOARD}/nuttx-export
        COMMAND unzip ${BOARD}.export -d ${CMAKE_BINARY_DIR}/NuttX-${BOARD}
        COMMAND touch ${CMAKE_BINARY_DIR}/NuttX-${BOARD}/nuttx-export/.stamp
        DEPENDS ${BOARD}.export)
    add_custom_target(NuttX_${BOARD}
        DEPENDS ${CMAKE_BINARY_DIR}/NuttX-${BOARD}/nuttx-export/.stamp)

    # this symbolic linking is to allow cmake to build all of the
    # archives for the current makefile, it won't be needed in a
    # complete cmake build
    add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        COMMAND mkdir -p ${CMAKE_SOURCE_DIR}/Archives
        COMMAND rm -f ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        COMMAND ln -sf ${CMAKE_BINARY_DIR}/${BOARD}.export ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export
        DEPENDS ${BOARD}.export)
    add_custom_target(link_export_${BOARD}
        DEPENDS ${CMAKE_SOURCE_DIR}/Archives/${BOARD}.export)
endmacro()

# flags
#-----------------------------------------------------
if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
    # board CORTEXM44
    set(CPU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -march=armv7e-m
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        )
else()
    set(CPU_FLAGS)
endif()
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
    #-Wcast-qual  - generates spurious noreturn attribute warnings, try again later
    #-Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
    #-Wcast-align - would help catch bad casts in some cases, but generates too many false positives
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

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/.git)
    set(GIT_REPO True)
else()
    set(GIT_REPO False)
endif()

message(STATUS "CMAKE CURRENTLY CAN ONLY BUILD THE NUTTX ARCHIVES: (target) link_exports")
message(STATUS "cflags: ${CMAKE_C_FLAGS}")

set(NUTTX_SRC "${CMAKE_SOURCE_DIR}/NuttX")

# git submodules
#---------------------------------------------------------
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
    add_custom_command(OUTPUT git_${MOD_NAME}.stamp
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND git submodule init ${${MOD}_path}
        COMMAND git submodule update ${${MOD}_path}
        COMMAND touch ${CMAKE_BINARY_DIR}/git_${MOD}.stamp
        )
    add_custom_target(git_${MOD_NAME}
        DEPENDS git_${MOD_NAME}.stamp
        )
endforeach()

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

# Firmware
if (FIRMWARE_BUILD)
    foreach(BOARD IN LISTS BOARDS)
        message(STATUS ${BOARD})
        set(EXPORT_DIR ${CMAKE_BINARY_DIR}/NuttX-${BOARD}/nuttx-export/)
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
                message(STATUS \t\t${MODULE})
                # name for module
                set(MOD ${MODULE}_${BOARD}_${CONFIG})
                # define how a module is built
                add_library(${MOD} ${${MODULE}_srcs})
                # set NuttX for board as dep
                add_dependencies(${MOD} NuttX_${BOARD})
                # add module to list of modules for board config
                list(APPEND MODULES ${MOD})
            endforeach()

            # build the executable
            link_directories(${EXPORT_DIR}/libs)
            link_libraries(
                ${MODULES}
                apps
                nuttx)
            file(GLOB STARTUP_OBJS ${EXPORT_DIR}/startup/*.oa)
            message(STATUS objs: ${STARTUP_OBJS})
            set_source_files_properties(${STARTUP_OBJS}
                PROPERTIES
                EXTERNAL_OBJECT TRUE  # Identifies this as an object file
                GENERATED TRUE  # Avoids need for file to exist at configure-time
                )
            add_executable(${BOARD}_${CONFIG} cmake/empty.c ${STARTUP_OBJS})
            # generate the firmware
            if (NOT HOST_TEST)
                set_target_properties(${BOARD}_${CONFIG} PROPERTIES
                    LINK_FLAGS "-T${EXPORT_DIR}/build/ld.script"
                    #COMPILE_FLAGS "-Werror"
                )
                generate_firmware(${BOARD}_${CONFIG})
            endif()

        endforeach()
    endforeach(BOARD IN LISTS BOARDS)
endif(FIRMWARE_BUILD)

# vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 ft=cmake:
