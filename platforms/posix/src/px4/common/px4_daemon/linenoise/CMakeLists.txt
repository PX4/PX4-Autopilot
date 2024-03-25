cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 99)

project(linenoise)

add_library(linenoise
    linenoise.c
)

target_compile_options(linenoise PRIVATE "-fsigned-char")

target_include_directories(linenoise PUBLIC
    .
)
