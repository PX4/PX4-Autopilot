What we're doing: Removing CRTP template bloat from PX4's ModuleBase<T>. The old pattern instantiates ~6 static methods (main, start, stop, status, exit_and_cleanup, run_trampoline) per module — nearly
   identical code duplicated 96 times. We replace it with a non-template ModuleBase class + per-module Descriptor struct that holds function pointers and storage.

  What we did this session:
  1. Created module_base.h (non-template class + Descriptor) and module_base.cpp (shared implementations — 1,018 bytes, one copy)
  2. Added it to the px4_platform library in CMakeLists.txt
  3. Converted 5 modules: TimePersistor, EscBattery, LoadMon, ToneAlarm (work-queue), and GPS (thread-based with secondary instance)
  4. Fixed two build issues: MODULE_NAME not defined in platform library context, and -Wshadow on lambda parameters
  5. Verified clean builds on both ark_fmu-v6x_default and px4_sitl_default
  6. Measured flash savings: 1,576 bytes from 4 modules, ~680 bytes per module, projected ~64 KB for all 96

  What's left: ~91 more modules to convert (mechanical for work-queue modules, slightly more involved for thread-based ones). The memory file module-base-crtp-removal.md has the full conversion pattern
  and gotchas for the next session.
