#pragma once

#include <platforms/posix/apps.h>

#include <functional>

std::function<void(apps_map_type &apps)> stub_init_app_map_callback = [](apps_map_type &) {};
std::function<void(apps_map_type &apps)> stub_list_builtins_callback = [](apps_map_type &) {};
