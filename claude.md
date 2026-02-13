# PX4 Flash Size Reduction

## Context

PR #26462 proposes enabling LTO for PX4-Autopilot NuttX targets, claiming ~100KB flash savings. However, LTO strips `__attribute__((section(".name")))` which breaks TCM relocation on STM32F7/H7 boards. We want to recover as much of that 100KB as possible without LTO.

## Target Board

`ark_fmu-v6x_default` — all builds and measurements use this target.

## Baseline (measured)

- `.text`: 1,960,864 B
- `.data`: 4,720 B
- **Total FLASH: 1,965,952 B (99.99% of 1,920 KB) — 128 bytes free**
- Build type: MinSizeRel (`-Os`)
- Build command: `make ark_fmu-v6x_default`
- bloaty is installed at `/tmp/bloaty-src/build/bloaty`

## Investigation Completed (Steps 1-8)

All build system flags are already correctly configured:
- `-ffunction-sections`, `-fdata-sections`, `-Wl,--gc-sections`: all applied ✅
- `-fvisibility=hidden`: already set ✅
- `-fno-rtti`, `-fno-exceptions`, `-fno-threadsafe-statics`: all set ✅
- gc-sections actively removes 4,547 text + 1,108 rodata sections ✅
- COMDAT folding works — zero duplicate symbols at different addresses ✅

### Top Flash Consumers (bloaty VM size)

| Component | Size | Notes |
|-----------|------|-------|
| EKF2 (unity build) | 162 KB | Legitimate — EKF math |
| ROMFS (g_cromfs_image) | 156 KB | See ROMFS breakdown below |
| Mavlink (unity build) | 112 KB | 93 stream send() methods |
| uavcan templates | 114 KB | 3rd-party, 1801 symbols |
| matrix templates | 80 KB | Legitimately different sizes |
| UXRCE-DDS client | 40 KB | on_topic_update 7KB alone |
| CMSIS DSP tables | 36 KB | realCoefAQ15/BQ15 for gyro_fft |
| uORB | 32 KB | Includes 19KB compressed_fields |
| ModuleBase\<T\> CRTP | 28 KB | 96 modules, duplicated boilerplate |
| NuttX STM32 serial | 32 KB | Platform driver |

### ROMFS Breakdown (156 KB — essentially at minimum)

| Content | Size | % |
|---------|------|---|
| parameters.json.xz | 69 KB | 44% |
| px4_io-v2_default.bin | 39 KB | 25% |
| all_events.json.xz | 16 KB | 10% |
| Airframe files (29) | 30 KB | 19% |
| Init scripts (23) | 26 KB | — |
| actuators.json.xz | 3 KB | — |

All major items are necessary or already compressed. ROMFS is not a viable optimization target.

### Decisions Made

- **NOT setting CONSTRAINED_FLASH** — has other implications beyond size
- **NOT removing ROMFS content** — parameters.json.xz, px4_io-v2_default.bin, events are all necessary
- **NOT pursuing toolchain upgrade** for now (ICF requires gold/lld linker)

## Active Work Items

### 1. Investigate uORB::compressed_fields removal (19 KB potential savings)

**What it is:** A heatshrink-compressed byte array containing field definitions for ALL uORB message types. Generated at build time by `Tools/msg/px_generate_uorb_compressed_fields.py`. Lives in generated file `build/.../msg/topics_sources/uORBMessageFieldsGenerated.cpp`.

**What uses it:**
- `orb_print_message_internal()` in `platforms/common/uORB/uORB.cpp` — decompresses and pretty-prints topic data
- `listener` shell command (`src/systemcmds/topic_listener/`) — calls orb_print_message_internal
- `orb status` shell command
- Each generated topic .cpp has a `print_message()` function that calls orb_print_message_internal

**What does NOT use it:**
- Logger/ulog — uses its own format definitions, does NOT touch compressed_fields
- MAVLink — has its own message definitions
- Any flight-critical code path

**Open question:** Is losing `listener` and `orb status` on-device acceptable for a production build? These are debug/development tools. If so, this is a clean 19-21 KB savings (compressed_fields array + orb_print_message_internal code).

**Key files:**
- `platforms/common/uORB/uORBMessageFields.cpp` (lines 54-55) — runtime access via MessageFormatReader
- `platforms/common/uORB/uORB.cpp` (line 241-502) — orb_print_message_internal
- `msg/CMakeLists.txt` (lines 373-383) — generation build target
- `Tools/msg/px_generate_uorb_compressed_fields.py` — generator script

**TODO:** Determine if we want to proceed. If yes, add a Kconfig / cmake option to exclude compressed_fields and stub out orb_print_message_internal. Measure actual savings with a clean build.

### 2. De-template ModuleBase\<T\> CRTP pattern (~20-25 KB potential savings)

**What it is:** `ModuleBase<T>` in `platforms/common/include/px4_platform_common/module.h` uses CRTP so each of the 96 derived modules gets its own copy of `main()`, `stop_command()`, `status_command()`, etc. These are nearly identical — e.g., 46 copies of `main()` at ~296 bytes each = 13 KB just for that one method.

**Why it's templated (the only real reason):** Per-module static storage of `_object` (atomic pointer to singleton instance) and `_task_id`. The template parameter T gives each module its own static variables without a registry.

**Static methods that use T (cannot be virtual — called before object exists):**
- `main(int argc, char *argv[])` — dispatcher: parses start/stop/status, calls T::task_spawn(), T::custom_command(), T::print_usage()
- `run_trampoline()` — calls T::instantiate()
- `start_command_base()` — calls T::task_spawn()
- `stop_command()` — reads _object, calls request_stop()
- `status_command()` — reads _object, calls print_status()
- `get_instance()` — returns T* from _object

**Virtual methods (already virtual, no change needed):**
- `run()`, `print_status()`, `request_stop()`

**Required per-module static methods (called via CRTP):**
- `T::task_spawn(int argc, char *argv[])`
- `T::instantiate(int argc, char *argv[])`
- `T::custom_command(int argc, char *argv[])`
- `T::print_usage(const char *reason)`

**Proposed approach — module registry pattern:**

Replace per-template static storage with a registry keyed by module name. Each module registers a static descriptor struct containing function pointers:

```cpp
// Non-template base
class ModuleBase {
public:
    struct ModuleDescriptor {
        const char *name;
        int (*task_spawn)(int argc, char *argv[]);
        int (*custom_command)(int argc, char *argv[]);
        int (*print_usage)(const char *reason);
        px4::atomic<ModuleBase *> object{nullptr};
        int task_id{-1};
    };

    // Single non-template main() — finds descriptor by argv[0] or registered name
    static int main(ModuleDescriptor &desc, int argc, char *argv[]);

    // Instance methods (unchanged)
    virtual void run() {}
    virtual int print_status();
    void request_stop();
    bool should_exit() const;

protected:
    void exit_and_cleanup(ModuleDescriptor &desc);
    px4::atomic_bool _task_should_exit{false};
};

// Per-module: just declare a static descriptor + thin wrapper
// In header:
class MyModule : public ModuleBase, public ModuleParams {
public:
    static ModuleDescriptor desc;
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason);
    // ...
};

// In cpp:
MyModule::ModuleDescriptor MyModule::desc{"my_module", task_spawn, custom_command, print_usage};
int my_module_main(int argc, char *argv[]) { return ModuleBase::main(MyModule::desc, argc, argv); }
```

**This approach:**
- `main()`, `stop_command()`, `status_command()` collapse from 96 copies to 1 each (~20-25 KB savings)
- Per-module static storage moves into the descriptor struct (still per-module, no global registry needed)
- `get_instance()` returns `ModuleBase*` instead of `T*` — callers need a static_cast, but most internal callers already have the concrete type
- Each module keeps its own static `task_spawn`/`custom_command`/`print_usage` functions (no change to those)
- The `_main()` entry point per module just passes its descriptor to the shared `ModuleBase::main()`

**Key challenge:** `get_instance()` currently returns `T*`. Internal module code that calls `get_instance()` would need updating. Grep for `get_instance()` usage to quantify the scope.

**Estimated scope:** 96 modules to update (mostly mechanical: add descriptor, change inheritance, update _main entry point). Can be done incrementally — keep both template and non-template versions during transition.

**Key files:**
- `platforms/common/include/px4_platform_common/module.h` — ModuleBase<T> definition
- `src/templates/template_module/` — canonical example module
- All 96 modules inheriting ModuleBase

## Build & Measurement Notes

- Do not modify `platforms/nuttx/NuttX/` — that's a submodule
- Always do clean builds when measuring: `make clean && make ark_fmu-v6x_default`
- Use `arm-none-eabi-size` and/or `arm-none-eabi-nm` on the output ELF
- Map file is auto-generated at `build/ark_fmu-v6x_default/ark_fmu-v6x_default.map`
- bloaty: `/tmp/bloaty-src/build/bloaty`
