cmake_minimum_required(VERSION 3.5)
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
                $<$<C_COMPILER_ID:MSVC>:/wd4018>
                $<$<C_COMPILER_ID:MSVC>:/wd4061>
                $<$<C_COMPILER_ID:MSVC>:/wd4062>
                $<$<C_COMPILER_ID:MSVC>:/wd4100>
                $<$<C_COMPILER_ID:MSVC>:/wd4101>
                $<$<C_COMPILER_ID:MSVC>:/wd4189>
                $<$<C_COMPILER_ID:MSVC>:/wd4389>
                $<$<C_COMPILER_ID:MSVC>:/wd4390>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-double-promotion>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-pointer-sign>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-cast-align>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-unused-but-set-parameter>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-sign-compare>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-bad-function-cast>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-empty-body>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-switch>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-unused-variable>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-format>
                $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wno-format-security>
)

if("${PX4_PLATFORM}" MATCHES "nuttx")
    target_compile_definitions(libvnc PUBLIC __NUTTX__)
endif()
