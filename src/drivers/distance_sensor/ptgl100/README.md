# PTGL-100 Driver Validation

Run the following commands from the PX4-Autopilot repository root.

## Firmware Build

Build the Flycore firmware:

```sh
make amovlab_flycore_default
```

## Parser Unit Tests

Configure and run the driver-specific SITL tests with a current CMake installation:

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DTESTFILTER=PTGL100Parser
cmake --build build/px4_sitl_test --target px4 unit-PTGL100Parser
ctest --test-dir build/px4_sitl_test -R '^unit-PTGL100Parser$' --output-on-failure
```

## Virtual UART Integration Test

Run the integration test against the SITL test executable built above:

```sh
python3 src/drivers/distance_sensor/ptgl100/test_ptgl100.py --px4 build/px4_sitl_test/bin/px4
```

The integration test requires `pexpect`, `pymavlink`, and `pyulog`.
It uses a pseudo-terminal in a temporary directory and does not require a connected sensor or flight controller.
