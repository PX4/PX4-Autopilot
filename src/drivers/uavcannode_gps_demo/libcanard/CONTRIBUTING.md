# Libcanard contribution guide

## Design principles

The library is intended for use in real-time high-integrity applications.
It is paramount that its temporal properties and resource utilization are plain to model and predict statically.
The code shall follow applicable high-reliability coding guidelines as explained later in this document.
The implementation shall be fully compliant with the UAVCAN/CAN specification.

The implementation and the API should be kept simple.
The core library `canard.c` (that is, excluding the optional DSDL presentation layer extension) shall never become
larger than 1000 logical lines of code.
This restriction ensures that the library is kept simple and easy to validate and verify.
There will be no high-level abstractions -- if that is desired, other implementations of UAVCAN should be used.

The library is intended for deeply embedded systems where the resources may be scarce.
The ROM footprint is of a particular concern because the library should be usable with embedded bootloaders.

## Directory layout

The production sources and some minimal configuration files (such as `.clang-tidy`) are located under `/libcanard/`.
Do not put anything else in there.

The tests are located under `/tests/`.
This directory also contains the top `CMakeLists.txt` needed to build and run the tests on the local machine.

There is no separate storage for the documentation because it is written directly in the header files.
This works for Libcanard because it is sufficiently compact and simple.

## Standards

The library shall be implemented in ISO C99/C11 following MISRA C:2012.
The MISRA compliance is enforced by Clang-Tidy and SonarQube.
Deviations are documented directly in the source code as follows:

```c
// Intentional violation of MISRA: <some valid reason>
<... deviant construct ...>
```

The full list of deviations with the accompanying explanation can be found by grepping the sources.

Do not suppress compliance warnings using the means provided by static analysis tools because such deviations
are impossible to track at the source code level.
An exception applies for the case of false-positive (invalid) warnings -- those should not be mentioned in the codebase.

[Zubax C++ Coding Conventions](https://kb.zubax.com/x/84Ah) shall be followed.
Formatting is enforced by Clang-Format; it is used also to fail the CI/CD build if violations are detected.

Unfortunately, some rules are hard or impractical to enforce automatically,
so code reviewers shall be aware of MISRA and general high-reliability coding practices
to prevent non-compliant code from being accepted into upstream.

## Tools

The following tools are required to conduct library development locally:

- GCC v10 or newer.
- Clang and Clang-Tools v11 or newer.
- CMake v3.12 or newer.
- An AMD64 machine.

### Clang-Tidy

Clang-Tidy is used to enforce compliance with MISRA and Zubax Coding Conventions.
There are separate configuration files per directory:

- `/libcanard/` (the production code) is equipped with the most stringent configuration.

- `/tests/` is equipped with a separate configuration which omits certain rules that are considered
  expensive to maintain.
  This is because the test suite is intentionally kept to a somewhat lower quality bar to reduce development costs.

Clang-Tidy is invoked automatically on each translation unit before it is compiled;
the build will fail if the tool is not available locally.
To disable this behavior, pass `NO_STATIC_ANALYSIS=1` to CMake at the generation time.

### Clang-Format

Clang-Format is used to enforce compliance with MISRA and Zubax Coding Conventions.
There is a single configuration file at the root of the repository.

To reformat the sources, generate the project and build the target `format`; e.g., for Make: `make format`.

### SonarQube

SonarQube is a cloud solution so its use is delegated to the CI/CD pipeline.
If you need access, please get in touch with the UAVCAN Development Team members.

### IDE

The recommended development environment is JetBrains CLion. The root project file is `tests/CMakeLists.txt`.
The repository contains the spelling dictionaries for CLion located under `.idea/`, make sure to use them.

## Testing

Generate the CMake project, build all, and then build the target `test` (e.g., `make test`).
The tests are built for x86 and x86_64; the latter is why an AMD64 machine is required for local development.

At the moment, the library is not being tested against other platforms.
We would welcome contributions implementing CI/CD testing against popular embedded architectures, particularly
the ARM Cortex M series and AVR in an emulator.
As a high-integrity library, the Libcanard test suite should provide full test coverage for all commonly used platforms.

## Releasing

Simply create a new release on GitHub: <https://github.com/UAVCAN/libcanard/releases/new>
