/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include <iostream>
#include <iomanip>
#include "debug.hpp"

int main(int argc, const char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        ENFORCE(0 == std::system("mkdir -p /tmp/uavcan_posix/dynamic_node_id_server"));

        /*
         * Event tracer test
         */
        {
            using namespace uavcan::dynamic_node_id_server;

            const std::string event_log_file("/tmp/uavcan_posix/dynamic_node_id_server/event.log");

            uavcan_posix::dynamic_node_id_server::FileEventTracer tracer;
            ENFORCE(0 <= tracer.init(event_log_file.c_str()));

            // Adding a line
            static_cast<IEventTracer&>(tracer).onEvent(TraceError, 123456);
            ENFORCE(0 == std::system(("cat " + event_log_file).c_str()));

            // Removing the log file
            ENFORCE(0 == std::system(("rm -f " + event_log_file).c_str()));

            // Adding another line
            static_cast<IEventTracer&>(tracer).onEvent(TraceError, 789123);
            ENFORCE(0 == std::system(("cat " + event_log_file).c_str()));
        }

        /*
         * Storage backend test
         */
        {
            using namespace uavcan::dynamic_node_id_server;

            uavcan_posix::dynamic_node_id_server::FileStorageBackend backend;
            ENFORCE(0 <= backend.init("/tmp/uavcan_posix/dynamic_node_id_server/storage"));

            auto print_key = [&](const char* key) {
                std::cout << static_cast<IStorageBackend&>(backend).get(key).c_str() << std::endl;
            };

            print_key("foobar");

            static_cast<IStorageBackend&>(backend).set("foobar", "0123456789abcdef0123456789abcdef");
            static_cast<IStorageBackend&>(backend).set("the_answer", "42");

            print_key("foobar");
            print_key("the_answer");
            print_key("nonexistent");
        }

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
