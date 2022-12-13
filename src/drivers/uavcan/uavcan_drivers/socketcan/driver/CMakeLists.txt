include_directories(
    ./include
    )

add_compile_options(-Wno-unused-variable)
add_library(uavcan_socketcan_driver STATIC
    src/socketcan.cpp
    src/thread.cpp
    src/clock.cpp
    )

add_dependencies(uavcan_socketcan_driver uavcan)

install(DIRECTORY include/uavcan_nuttx DESTINATION include)
install(TARGETS uavcan_socketcan_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :)
