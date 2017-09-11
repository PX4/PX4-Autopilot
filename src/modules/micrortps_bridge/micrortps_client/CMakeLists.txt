############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

if(NOT GENERATE_RTPS_BRIDGE MATCHES "off")

    find_program(FASTRTPSGEN fastrtpsgen
                 PATHS $ENV{FASTRTPSGEN_DIR})
    if(NOT FASTRTPSGEN)
        message(WARNING "WARNING: Unable to find fastrtpsgen. Building PX4 without RTPS bridge support")
        px4_add_module(
            MODULE modules__micrortps_bridge__micrortps_client
            MAIN micrortps_client
            SRCS
                microRTPS_client_dummy.cpp
            DEPENDS
                platforms__common
        )
        return()
    endif()

    include_directories(.)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    # Do not delete everything in the current dir
    set(msg_out_path ${CMAKE_CURRENT_BINARY_DIR})
    set(topic_msg_path ${PX4_SOURCE_DIR}/msg)

    set(send_topic_files)
    foreach(topic ${config_rtps_send_topics})
        list(APPEND send_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
    endforeach()

    set(receive_topic_files)
    foreach(topic ${config_rtps_receive_topics})
        list(APPEND receive_topic_files ${PX4_SOURCE_DIR}/msg/${topic}.msg)
    endforeach()

    foreach(topic ${config_rtps_send_topics})
        list(APPEND topic_bridge_files_out ${msg_out_path}/${topic}_Publisher.cpp)
        list(APPEND topic_bridge_files_out ${msg_out_path}/${topic}_Publisher.h)
    endforeach()

    foreach(topic ${config_rtps_receive_topics})
        list(APPEND topic_bridge_files_out ${msg_out_path}/${topic}_Subscriber.cpp)
        list(APPEND topic_bridge_files_out ${msg_out_path}/${topic}_Subscriber.h)
    endforeach()

    if (NOT "${config_rtps_send_topics}" STREQUAL "" OR NOT "${config_rtps_receive_topics}" STREQUAL "")
        set(topic_bridge_files_out microRTPS_client.cpp microRTPS_transport.cpp microRTPS_transport.h)

        set(send_topic_files_opt)
        if (NOT "${send_topic_files}" STREQUAL "")
            set(send_topic_opt "-s")
        endif()

        set(receive_topic_files_opt)
        if (NOT "${receive_topic_files}" STREQUAL "")
            set(receive_topic_opt "-r")
        endif()

        add_custom_command(OUTPUT ${topic_bridge_files_out}
            COMMAND ${PYTHON_EXECUTABLE}
            ${PX4_SOURCE_DIR}/Tools/generate_microRTPS_bridge.py
            -f $ENV{FASTRTPSGEN_DIR}
            ${send_topic_opt} ${send_topic_files}
            ${receive_topic_opt} ${receive_topic_files}
            -t ${topic_msg_path}
            -u ${msg_out_path}
            DEPENDS ${DEPENDS} ${send_topic_files} ${receive_topic_files}
            COMMENT "Generating RTPS topic bridge"
            VERBATIM
            )
    endif()

endif()

px4_add_module(
    MODULE modules__micrortps_bridge__micrortps_client
    MAIN micrortps_client
    STACK_MAIN 4096
    SRCS
        microRTPS_transport.cpp
        microRTPS_client.cpp
        microRTPS_client.h
        microRTPS_client_main.cpp
    DEPENDS
        platforms__common
    )

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
