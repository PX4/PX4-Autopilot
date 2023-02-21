cmake_minimum_required(VERSION 2.8.4)
project(libvnc C)

include_directories(include)

set(SOURCES
        src/vn/xplat/criticalsection.c
        src/vn/xplat/event.c
        src/vn/xplat/serialport.c
        src/vn/xplat/thread.c
        src/vn/xplat/time.c
        src/vn/error.c
        src/vn/error_detection.c
        src/vn/util.c
        src/vn/math/matrix.c
        src/vn/math/vector.c
        src/vn/protocol/upack.c
        src/vn/protocol/upackf.c
        src/vn/protocol/spi.c
        src/vn/sensors/compositedata.c
        src/vn/sensors/searcher.c
        src/vn/sensors.c
        src/vn/sensors/ezasyncdata.c
        src/vn/conv.c
        include/vn/vectornav.h
        include/vn/bool.h
        include/vn/util/compiler.h
        include/vn/sensors/compositedata.h
        include/vn/const.h
        include/vn/conv.h
        include/vn/xplat/criticalsection.h
        include/vn/enum.h
        include/vn/error_detection.h
        include/vn/error.h
        include/vn/xplat/event.h
        include/vn/util/export.h
        include/vn/sensors/ezasyncdata.h
        include/vn/int.h
        include/vn/math/matrix.h
        include/vn/util/port.h
        include/vn/protocol/common.h
        include/vn/sensors/searcher.h
        include/vn/sensors.h
        include/vn/xplat/serialport.h
        include/vn/protocol/spi.h
        include/vn/xplat/thread.h
        include/vn/xplat/time.h
        include/vn/types.h
        include/vn/protocol/upack.h
        include/vn/protocol/upackf.h
        include/vn/util.h
        include/vn/math/vector.h
)

add_library(libvnc ${SOURCES})
add_dependencies(libvnc prebuild_targets)
target_compile_options(libvnc
        PRIVATE
                -Wno-error
                -Wno-double-promotion
                -Wno-pointer-sign
                -Wno-cast-align
                -Wno-unused-but-set-parameter
                -Wno-sign-compare
                -Wno-bad-function-cast
                -Wno-empty-body
                -Wno-switch
                -Wno-unused-variable
                -Wno-format
                -Wno-format-security
)

if("${PX4_PLATFORM}" MATCHES "nuttx")
    target_compile_definitions(libvnc PUBLIC __NUTTX__)
endif()
