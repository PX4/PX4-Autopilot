find_package(Threads)


if(RL_TOOLS_ENABLE_JSON AND RL_TOOLS_ENABLE_BOOST_BEAST AND RL_TOOLS_ENABLE_CLI11 AND Threads_FOUND)
add_executable(ui_server server.cpp)
target_link_libraries(
        ui_server
        PRIVATE
        RLtools::CoreDefinitions
        nlohmann_json::nlohmann_json
        Boost::filesystem
        CLI11::CLI11
        Threads::Threads
)

endif()
