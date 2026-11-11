# Turtle Recovery Tests

Unit tests cover the controller and supporting headers:

- `DragonflyTurtleCore.hpp`: recovery state machine, output ramping and lifecycle.
- `DragonflyTurtleRc.hpp`: RC timestamp and channel validity.
- `DragonflyTurtleInput.hpp`: switch selection and channel conflicts.
- `DragonflyTurtleMapping.hpp`: motor functions, geometry and output masks.
- `ArmingInterlock.hpp`: flight/recovery ownership and guard lifetimes.

The lifecycle suites use synthetic RC samples and controller ticks to exercise input cadence and driver lifecycle conditions.

## Standalone Tests

With a host C++ compiler:

```sh
python3 test/turtle_recovery/run_host_tests.py
```

The runner enables UndefinedBehaviorSanitizer and warnings as errors, and keeps assertions active even when testing release configuration.
`CXX` may select another compatible compiler executable.

## PX4 Native Test Graph

With the PX4 native build dependencies installed:

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DTESTFILTER='^unit-turtle_recovery_'
cmake --build build/px4_sitl_test --target unit-turtle_recovery_rc unit-turtle_recovery_core unit-turtle_recovery_fault_matrix unit-turtle_recovery_mapping unit-turtle_recovery_interlock unit-turtle_recovery_lifecycle
ctest --test-dir build/px4_sitl_test -R '^unit-turtle_recovery_' --output-on-failure
```

Do not run native and NuttX builds concurrently in one source tree.
These targets are part of the PX4 CMake test graph, with assertions explicitly kept active.

## Coverage

- RC timestamp validity, optional last-valid timestamp, channel selection and function conflicts.
- Controller entry, output selection, ramping, timeout, cancellation, restoration and release.
- Phase/fault combinations and repeated recovery/flight-ownership cycles.
- Logical motor/output permutations and mask boundaries.
- Cross-translation-unit interlock identity and competing flight/recovery reservations.
- Startup and command arbitration, disable/recreate lifecycle, missing/malformed switch channels, callback gaps during OFF qualification and stationary-control cadence.
- Startup restoration with recovery disabled and retained ESC direction in the reset model.
