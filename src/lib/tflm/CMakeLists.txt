############################################################################
#
#   Copyright (c) 2017-2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_git_submodule(TARGET git_tflite-micro PATH tflite_micro)

set(TFLITE_DOWNLOADS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tools/make/downloads)
set(TFLITE_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/gen/cortex_m_generic_cortex-m7_default_cmsis_nn_gcc)

get_directory_property(FLAGS COMPILE_OPTIONS)
list(REMOVE_ITEM FLAGS "-Wcast-align")
set_directory_properties(PROPERTIES COMPILE_OPTIONS "${FLAGS}")


file(GLOB TFLITE_MICRO_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/arena_allocator/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/memory_planner/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tflite_bridge/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/kernels/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/internal/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/api/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/c/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/schema/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/core/api/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/schema/*.cc
)

# Filter out tests as they cause errors
list(FILTER TFLITE_MICRO_SRCS EXCLUDE REGEX ".*_test.*\\.cc$")

set(TFLM_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro
    ${TFLITE_DOWNLOADS_DIR}
    ${TFLITE_DOWNLOADS_DIR}/flatbuffers/include
    ${TFLITE_DOWNLOADS_DIR}/ruy
    ${TFLITE_DOWNLOADS_DIR}/gemmlowp
    ${TFLITE_DOWNLOADS_DIR}/kissfft
    ${TFLITE_DOWNLOADS_DIR}/cmsis/Cortex_DFP/Device/ARMCM7/Include
    ${TFLITE_DOWNLOADS_DIR}/cmsis/CMSIS/Core/Include
    ${TFLITE_DOWNLOADS_DIR}/cmsis
    ${TFLITE_DOWNLOADS_DIR}/cmsis_nn
    ${TFLITE_DOWNLOADS_DIR}/cmsis_nn/Include
    ${TFLITE_GEN_DIR}/genfiles
    )

    # Add C++ 17 std lib if building for NuttX
    if(CONFIG_BOARD_TOOLCHAIN STREQUAL "arm-none-eabi")
    list(APPEND TFLM_INCLUDE_DIRS
	${TFLITE_DOWNLOADS_DIR}/include/13.2.1
        ${TFLITE_DOWNLOADS_DIR}/include/13.2.1/arm-none-eabi
    )
endif()

px4_add_library(tflm ${TFLITE_MICRO_SRCS})

target_include_directories(tflm PUBLIC ${TFLM_INCLUDE_DIRS})

target_compile_options(tflm PUBLIC
    -Wno-float-equal
    -Wno-shadow
)
