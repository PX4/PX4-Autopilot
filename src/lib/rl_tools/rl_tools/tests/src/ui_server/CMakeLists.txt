

find_package(Threads)
if(RL_TOOLS_ENABLE_JSON AND RL_TOOLS_ENABLE_BOOST_BEAST AND Threads_FOUND)
add_executable(test_ui_server_client client.cpp)

target_link_libraries(
    test_ui_server_client
    PRIVATE
    RLtools::Core
    Boost::boost
    nlohmann_json::nlohmann_json
    Threads::Threads
)
endif()