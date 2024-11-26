#include <px4_platform_common/module_manager.h>
#include <unordered_map>

class ModuleManagerImpl {
public:
  void register_module(const ModuleEntry &entry) {
    modules.push_back(entry);
    by_name[entry.name] = entry;
  }

  std::vector<ModuleEntry> modules;
  std::unordered_map<std::string, ModuleEntry> by_name;

  friend class ModuleManager;
};
ModuleManagerImpl g_module;

void ModuleManager::register_module(const ModuleEntry &entry) { g_module.register_module(entry); }

std::vector<std::string> ModuleManager::get_running() { return {}; }

const std::vector<ModuleEntry> &ModuleManager::get_modules() { return g_module.modules; }

const ModuleEntry *ModuleManager::get_module(const std::string &name) {
  auto it = g_module.by_name.find(name);
  if (it == g_module.by_name.end()) return nullptr;
  return &it->second;
}

