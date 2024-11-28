#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <optional>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/px4_macros.h>

struct ModuleEntry {
  std::string name;
  int (*stop_command)();
  bool (*is_running)();
  void (*request_stop)();
};

class ModuleManager {
public:
  static void register_module(const ModuleEntry &entry);
  static std::vector<std::string> get_running();
  static void cleanup();
  static const std::vector<ModuleEntry> &get_modules();
  static const ModuleEntry* get_module(const std::string &name);
};

