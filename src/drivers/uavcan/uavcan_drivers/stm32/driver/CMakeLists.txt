include_directories(
    ./include
    )

add_library(uavcan_stm32_driver STATIC
    ./src/uc_stm32_can.cpp
    ./src/uc_stm32_clock.cpp
    ./src/uc_stm32_thread.cpp
    )

add_dependencies(uavcan_stm32_driver uavcan)

install(DIRECTORY include/uavcan_stm32 DESTINATION include)
install(TARGETS uavcan_stm32_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :)
