---
applyTo: ".github/**,cmake/**,Makefile,CMakeLists.txt,Tools/**,**/Kconfig"
---

# CI/Build Review Guidelines

In addition to the core code review guidelines:

- Check for pipeline race conditions (tag + branch push double-trigger, git describe correctness)
- Container image size: check for layer bloat
- Ubuntu LTS support policy: only latest + one prior LTS version
- Consider build time impact of changes
- Prefer CMake over Makefiles
