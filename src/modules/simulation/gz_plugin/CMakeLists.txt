cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)
find_package(gz-cmake3 REQUIRED)

project(OpticalFlowSystem)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-sensors8 REQUIRED)
gz_find_package(gz-transport12 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS optical_flow.proto)

add_library(${PROJECT_NAME} SHARED
    OpticalFlow.cpp
    OpticalFlowSystem.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC gz-sensors8::gz-sensors8
    PUBLIC gz-rendering8::gz-rendering8
    PUBLIC gz-plugin2::gz-plugin2
    PUBLIC gz-sim8::gz-sim8
    PUBLIC gz-transport12::gz-transport12
    PUBLIC ${OpenCV_LIBS}
    PUBLIC ${PROTOBUF_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)
