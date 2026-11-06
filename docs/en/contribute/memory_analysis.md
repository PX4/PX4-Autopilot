# Memory Analysis Reports

The Flash and RAM Analysis report compares two firmware builds for the same target and compiler.
Its additional views expose changes that linked section sizes alone cannot show.
The initial implementation is advisory: it does not certify a safe memory margin or enforce a runtime memory budget.

## Measurements

| View                  | Source                      | What It Establishes                                          | What It Does Not Establish                                                    |
| --------------------- | --------------------------- | ------------------------------------------------------------ | ----------------------------------------------------------------------------- |
| Flash                 | ELF load image              | Programmed bytes, including initialised data                 | Runtime RAM consumption                                                       |
| Static RAM            | ELF allocated RAM sections  | Storage reserved by the linker, including `.data` and `.bss` | Heap allocations or normal task stacks                                        |
| Function Stack Frames | GCC `-fstack-usage` records | Each compiled function's own frame                           | Maximum stack along a complete task call path                                 |
| Module Object Sizes   | Target ELF DWARF type sizes | Per-instance `sizeof` of explicitly tracked classes          | Total heap use, pointed-to allocations, allocator overhead or instance counts |

These measurements are separate views, not quantities to add together.
A member object is already included in its containing object's size.
On NuttX, task-stack allocations may also appear in heap accounting; adding those measurements again would double-count them.

## Why Module Object Sizes Matter

A 64-element array of two 32-bit coordinates requires 512 B of payload.
Moving that buffer from a local static into an inline Navigator-owned member transfers its storage from static RAM to Navigator's existing startup allocation.
The fixed object grows by the buffer size, subject to layout and alignment, while the static RAM report can decrease by 512 B.
This is a change of storage ownership, not a 512 B reduction in live RAM consumption while Navigator runs.

The member creates no additional allocation during a fence reload.
It enlarges the existing allocation when Navigator starts and is released with that object.
Actual allocation success still depends on available memory, allocation overhead and a sufficiently large free block.

`Tools/ci/module_memory_manifest.json` records the types whose allocation sites have been checked.
The initial set is Navigator, its embedded geofence avoidance planner, Commander and Logger.
The planner row explains the source of an owner-size change; it must not be added to the Navigator row.
Logger's separate buffers and subscription arrays are not included in its object size.
A missing or conflicting DWARF definition is reported as unavailable, not zero.

## Reading Stack Changes

GCC distinguishes fixed frames, bounded dynamic frames and dynamic frames without a compiler-proven upper bound.
For an unbounded dynamic frame, the reported number covers only the bounded portion.
The report therefore does not subtract that number from a fixed frame and call the result a total stack saving.

For example, a v6x analysis build of the geofence planner produced a record of 96 B plus dynamic storage, followed by an 88 B fixed record with the private-member buffer.
The separate 64-vertex calculation was 512 B of VLA payload; it was not a bound established by `.su` alone.
These numbers describe that implementation and build, not a task watermark measurement or a universal frame size across targets and compiler versions.

Function identity uses source path and compiler function name, excluding line number.
Conflicting records, such as an inline function compiled differently in several translation units, remain ambiguous.
Obvious incomplete compiler-supplied names also remain ambiguous, even when their recorded sizes match.
New or removed records do not receive an invented zero baseline.
Coverage reports the collected files and function identities.
It does not imply complete coverage of linked code: inlining, linker removal, assembly and external libraries affect which records exist.
Moving a buffer off the stack does not automatically reduce a task's configured stack reservation.
A later reduction of that reservation needs separate justification from the task's complete stack requirements.

## Build And Report Flow

The existing analysis workflow builds the change and its baseline with the same toolchain and diagnostic option.
`Tools/ci/memory_analysis.cmake` adds `-fstack-usage` through CMake's project-include hook without replacing the firmware's optimisation flags.
The hook is supplied to both revisions, including baselines that predate this reporter.
It covers the CMake compilation units to which the hook applies; NuttX's separate Make build and other external projects are not guaranteed to emit records.
The workflow verifies that its compiler cache restores `.su` files as well as object files.

Each build is collected before cleaning or checking out the other revision.
The reporter, manifest and hook are preserved outside the checkout so the same implementation reads both builds.
The result contains commit IDs and compiler identity; comparisons reject a different target, toolchain or manifest.
The selected board variant and complete build configuration should also be kept with any results shared outside this workflow.

The bot retains its existing section summary and Bloaty breakdown.
The GitHub PR comment is the primary report, with clickable per-target details containing the before/after tables.
The collapsed headings show change counts and warning indicators, so reviewers can identify increases without opening every target.
Opening a target shows the measurements; explanations, colour thresholds, coverage and revisions stay in a nested **How to read this** section.
Only changed stack records are listed: changed sizes or qualifiers, added/removed records, and changed ambiguous observations.
Unchanged dynamic frames and unchanged ambiguities are omitted, including when their line numbers move.
Changes caused by headers or inlining are retained even if the function's own source file was not edited.
These are changes between the two builds, not a source-diff filter or proof of a specific causal call path.

Byte-delta indicators reuse the existing bot thresholds: yellow above a 100 B increase, red above a 1,000 B increase, and green for a decrease greater than 100 B.
These are change-size thresholds, not task or heap safety budgets.
Changed unbounded or unavailable measurements receive a separate warning and no invented numeric delta.
The comment caps the stack table at 12 rows per target, prioritising unknown dynamic bounds and larger deltas.
Long function and source names are shortened in the comment, while the supporting comparison retains their complete names.
An artifact per target retains the complete raw snapshots and full changed-record comparison for unusually large changes.
The supporting HTML report lists the same changed functions and supports filtering and printing, with no external assets or network dependencies.

Local collection uses the following commands after the analysis build:

```sh
python3 Tools/ci/memory_report.py collect \
  --elf build/px4_fmu-v6x_flash-analysis/px4_fmu-v6x_flash-analysis.elf \
  --stack-root build/px4_fmu-v6x_flash-analysis \
  --source-root "$PWD" \
  --manifest Tools/ci/module_memory_manifest.json \
  --target px4_fmu-v6x_flash-analysis \
  --revision "$(git rev-parse HEAD)" \
  --toolchain "$(arm-none-eabi-gcc -dumpfullversion)" \
  --output after.json

python3 Tools/ci/memory_report.py compare \
  --before before.json --after after.json \
  --json-output memory-report.json \
  --markdown-output memory-report.md \
  --summary-output memory-summary.txt \
  --html-output memory-report.html
```

Use Python 3.10 or newer, `pyelftools` and the ARM GNU binutils used by the existing section reporter.
Create `before.json` from the baseline with the same configuration and preserved analysis tools.

## Further Improvements

The next step is reviewed, target-specific budgets for selected module object sizes and unusually large individual frames.
Keep these checks distinct from whole-task stack budgets.
A large inline member can then fail an analysis check before the firmware is started, even though the linker cannot reserve or validate that future heap allocation.

A later call-graph analysis could estimate stack along known paths, but recursion, indirect calls and unanalysed libraries must remain explicit unknowns.
Such an estimate must include the target's context and interrupt requirements before it can inform a task-stack budget.
Summing all function frames is not a valid estimate.

Target-based regression tests should record:

- Module startup success and allocation failures.
- Task stack high-water marks and minimum remaining stack during the exercised scenarios.
- Maximum allocated heap, minimum free heap and the largest free block.
- Repeated mission and fence loads, with representative limits, for multicopter, fixed-wing and VTOL configurations.

These tests measure exercised paths, not every possible execution.
POSIX simulation can exercise behaviour, but its allocator and stack figures do not establish the memory margin of a NuttX flight controller.
On the target, determine whether heap statistics already include task stacks before presenting combined numbers.

## References

- [GCC stack-usage records](https://gcc.gnu.org/onlinedocs/gcc/Developer-Options.html)
- [CMake project code injection](https://cmake.org/cmake/help/latest/variable/CMAKE_PROJECT_INCLUDE.html)
- [NuttX memory manager status](https://nuttx.apache.org/docs/latest/applications/nsh/commands.html#free-show-memory-manager-status)
