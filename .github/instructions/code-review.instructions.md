---
applyTo: "src/**,boards/**,platforms/**,msg/**,cmake/**,Makefile,CMakeLists.txt,Tools/**,.github/**"
---

# PX4 Code Review Guidelines

## Conventions

- PR titles must follow conventional commits: `type(scope): description` (see CONTRIBUTING.md)
- Types: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`
- Scope should match the primary area of changed files
- Append `!` before the colon for breaking changes
- Both squash merge and rebase merge are enabled; merge commits are disabled
- Commits should be atomic and independently revertable
- WIP or review-response commits should be squashed before merge

## Core Checks (always apply)

- **Correctness**: logic errors, off-by-ones, unhandled edge cases
- **Type safety**: int16 overflow, float/double promotion, unsigned subtraction, use `uint64_t` for absolute time
- **Initialization**: uninitialized variables, missing default construction
- **Buffer safety**: unchecked array access, stack allocation of large buffers, snprintf bounds
- **Magic numbers**: every numeric literal needs a named constant or justification
- **Framework reuse**: use PX4_ERR/WARN/INFO, existing libraries (AlphaFilter, SlewRate, RateControl), MAVLink constants from the library
- **Naming**: accurate, no unjustified abbreviations, current terminology (GPS -> GNSS for new code)
- **Unnecessary complexity**: can code be removed instead of added? Is there a simpler pattern?
- **Test coverage**: new features should include unit or integration tests; bug fixes should include regression tests where practical
- **Formatting**: `make format` / `make check_format` (astyle) for C/C++ files; `clang-tidy` clean
- **Coding style**: C/C++ must follow the PX4 coding style (https://docs.px4.io/main/en/contribute/code.html)
- **Necessity**: challenge every addition. Is this actually needed or just copied?
- **Architecture fit**: does the code live in the module that naturally owns the data? No unnecessary cross-module dependencies
- **Ecosystem impact**: consider QGC users, log analysis tools, and third-party integrations
