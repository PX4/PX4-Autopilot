
if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

add_subdirectory(nn)
add_subdirectory(rl)
add_subdirectory(ui_server)
add_subdirectory(nn_models)
