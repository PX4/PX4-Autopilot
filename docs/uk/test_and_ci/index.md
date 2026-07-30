# Тестування платформи та безперервна інтеграція

PX4 широко протестовано за допомогою модульних та інтеграційних тестів шляхом безперервної інтеграції.
Тестування польоту також відбувається командою розробників та широкою спільнотою.

Розділи тестування:

- [Test Flights](../test_and_ci/test_flights.md) - How to make test flights (e.g. to [test PRs](../contribute/code.md#pull-requests))
- [Hardware Bench Testing (px4bench)](../test_and_ci/bench_testing.md) - Automated verification on real flight-controller hardware: release qualification, hardware-in-the-loop flight testing, and production end-of-line checks
- [Unit Tests](../test_and_ci/unit_tests.md)
- [Sanitizers](../test_and_ci/sanitizers.md) - Build SITL with ASan/TSan to catch memory errors and data races
- [Continuous Integration (CI)](../test_and_ci/continous_integration.md)
- [Integration Testing](../test_and_ci/integration_testing.md)
  - [MAVSDK Integration Testing](../test_and_ci/integration_testing_mavsdk.md)
  - [PX4 ROS2 Interface Library Integration Testing](../test_and_ci/integration_testing_px4_ros2_interface.md)
- [Docker](../test_and_ci/docker.md)
- [Maintenance](../test_and_ci/maintenance.md)
