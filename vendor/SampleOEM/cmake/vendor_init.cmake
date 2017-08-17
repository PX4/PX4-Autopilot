set(NUTTX_CONFIG_DIR ${CMAKE_SOURCE_DIR}/vendor/${VENDOR}/nuttx/nuttx-configs)
include_directories(${CMAKE_SOURCE_DIR}/platforms/nuttx/boards)
MESSAGE("Set NUTTX_CONFIG_DIR to ${NUTTX_CONFIG_DIR}")
