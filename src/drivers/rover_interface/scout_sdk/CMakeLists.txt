# Include directories
include_directories(
  include/scout_sdk/
  include/scout_sdk/agilex_protocol
  include/scout_sdk/CAN
)

# Source files
set(SOURCES
  src/agilex_protocol/agilex_msg_parser_v2.c
  src/agilex_protocol/agilex_protocol_v2_parser.cpp
  src/CAN/SocketCAN.cpp
  src/Utilities.cpp
  src/ScoutRobot.cpp
)

# Build library
add_library(scout_sdk STATIC ${SOURCES})
target_include_directories(scout_sdk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(scout_sdk PRIVATE nuttx_arch)
