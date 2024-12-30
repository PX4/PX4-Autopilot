find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
	optical_flow.proto
)

# add_library(gz_px4_messages STATIC
#     ${PROTO_SRCS}
#     ${PROTO_HDRS}
# )

px4_add_library(gz_px4_messages
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_include_directories(gz_px4_messages
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gz_px4_messages
    PUBLIC ${PROTOBUF_LIBRARIES}
)
