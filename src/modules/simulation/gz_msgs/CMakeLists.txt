############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Find required packages
find_package(Protobuf)

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${Protobuf_INCLUDE_DIRS}
    )
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
