# PX4 Development

This section explains how to support new vehicle types and variants, modify flight algorithms, add new modes, integrate new hardware, and communicate with PX4 from outside the flight controller.

:::tip
This section is for software developers and (new) hardware integrators.
It is not needed if you're building an existing airframe or flying using a PX4 vehicle.
:::

It explains how to:

- Get a [minimum developer setup](../dev_setup/config_initial.md), [build PX4 from source](../dev_setup/building_px4.md) and deploy on [numerous supported autopilots](../flight_controller/index.md).
- Understand the [PX4 System Architecture](../concept/architecture.md) and other core concepts.
- Learn how to modify the flight stack and middleware:
  - Modify flight algorithms and add new [flight modes](../concept/flight_modes.md).
  - Support new [airframes](../dev_airframes/index.md).
- Learn how to integrate PX4 with new hardware:
  - Support new sensors and actuators, including cameras, rangefinders, etc.
  - Modify PX4 to run on new autopilot hardware.
- [Simulate](../simulation/index.md), [test](../test_and_ci/index.md) and [debug/log](../debug/index.md) PX4.
- Communicate/integrate with external robotics APIs.

## Key Developer Links

- [Support](../contribute/support.md): Get help using the [discussion boards](https://discuss.px4.io//) and other support channels.
- [Weekly Dev Call](../contribute/dev_call.md): A great opportunity to meet the PX4 dev team and discuss platform technical details (including pull requests, major issues, general Q&A).
- [Licences](../contribute/licenses.md): What you can do with the code (free to use and modify under terms of the permissive [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause)!)
- [Contributing](../contribute/index.md): How to work with our [source code](../contribute/code.md).
