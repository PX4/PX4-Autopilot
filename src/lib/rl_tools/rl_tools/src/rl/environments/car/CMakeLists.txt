
if(RL_TOOLS_ENABLE_GTK AND RL_TOOLS_ENABLE_JSON)
add_executable(rl_environments_car_training car.cpp)
target_link_libraries(rl_environments_car_training
        PRIVATE
        rl_tools
)
#target_compile_definitions(rl_environments_car_training PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
if(RL_TOOLS_ENABLE_GTK)
    target_include_directories(rl_environments_car_training PRIVATE ${GTK3_INCLUDE_DIRS})
    target_link_directories(rl_environments_car_training PRIVATE ${GTK3_LIBRARY_DIRS})
    target_link_libraries(rl_environments_car_training
            PRIVATE
            ${GTK3_LIBRARIES}
    )
endif()
endif()

if(RL_TOOLS_ENABLE_BOOST_BEAST AND RL_TOOLS_ENABLE_JSON)
add_executable(rl_environments_car_interactive car_interactive.cpp)
target_link_libraries(rl_environments_car_interactive
        PRIVATE
        RLtools::Backend
        Boost::boost
        nlohmann_json::nlohmann_json
        tensorboard_logger
)
target_compile_definitions(rl_environments_car_interactive PRIVATE RL_TOOLS_DISABLE_TENSORBOARD)
endif()

if(RL_TOOLS_ENABLE_GTK AND RL_TOOLS_ENABLE_SDL2)
add_executable(rl_environments_car_ui ui.cpp)
target_include_directories(rl_environments_car_ui PRIVATE ${GTK3_INCLUDE_DIRS})
target_link_directories(rl_environments_car_ui PRIVATE ${GTK3_LIBRARY_DIRS})
target_include_directories(rl_environments_car_ui PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(rl_environments_car_ui
        PRIVATE
        rl_tools
        ${GTK3_LIBRARIES}
        ${SDL2_LIBRARIES}
)
endif()

if(RL_TOOLS_ENABLE_GTK AND RL_TOOLS_ENABLE_SDL2)
add_executable(rl_environments_car_track car_track.cpp)
target_include_directories(rl_environments_car_track PRIVATE ${GTK3_INCLUDE_DIRS})
target_link_directories(rl_environments_car_track PRIVATE ${GTK3_LIBRARY_DIRS})
target_include_directories(rl_environments_car_track PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(rl_environments_car_track
        PRIVATE
        rl_tools
        ${GTK3_LIBRARIES}
        ${SDL2_LIBRARIES}
)
endif()
