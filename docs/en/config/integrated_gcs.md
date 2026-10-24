# Integrated Ground Control Stations

A GCS typically includes manual control sticks, a display, a computer, telemetry, video capture, and aerials for the control, video, and data links.
An integrated GCS combines these into one handheld or portable device.
For PX4, the device runs _QGroundControl_ (QGC).

The main benefit is that a single unit provides both manual control and full ground control station features, such as mission planning, telemetry, and video.
Having fewer components also simplifies setup and shortens preparation time before vehicle operation.

::: tip
An integrated GCS sends manual control inputs to PX4 in one of two ways, depending on the device:

- As MAVLink messages over the telemetry link, combining control and telemetry into one stream (see [Joystick Setup](../config/joystick.md)).
- As a standard RC signal, such as S.Bus, output from the GCS's air unit (see [Radio Setup](../config/radio.md)).

PX4 uses a single control source automatically.
If you have more than one, [Manual Control](../config/manual_control.md#px4-configuration) explains how to choose between them with [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE).
:::

## Types of Integrated GCS

### Handheld Controllers

A handheld controller is a small unit that the operator carries during vehicle operation.

![UXV Technologies SRoC](../../assets/peripherals/integrated_gcs/uxv_technologies_sroc_7.png)

The following units assert support for PX4:

- [CubePilot Herelink](https://docs.cubepilot.org/herelink/herelink-overview)

- [SIYI UniRC 10 Pro](https://shop.siyi.biz/products/siyi-unirc-10-pro)

- [UXV Technologies SRoC 7" Android/Linux/Windows](https://www.uxvtechnologies.com/ground-control-stations/sroc-windows)

- [UXV Technologies SRoC S26](https://www.uxvtechnologies.com/ground-control-stations/sroc-s26)

- [UXV Technologies Micronav](https://www.uxvtechnologies.com/ground-control-stations/micronav)

### Stationary, Transportable GCS

The form factor and weight of this type of GCS do not allow it to be moved during vehicle operation.
It is designed to be packed up easily and transported to a different location after operation.

![UXV Technologies G2Nav](../../assets/peripherals/integrated_gcs/uxv_technologies_g2_nav.webp)

GCS in this category:

- [Alpha Unmanned Systems G-Case Duo](https://www.alphaunmannedsystems.com/product/ground-control-station)

- [UXV Technologies G2Nav](https://www.uxvtechnologies.com/ground-control-stations/g2nav) (confirmed to work with PX4 and QGC)

## Selection Criteria

Factors to consider when selecting an integrated GCS:

- **Swappable Radio**: Some units allow the radio to be swapped easily, which slightly increases the weight of the system.

- **Operating System**: Devices are sold with Windows, Android, or Linux, and must be able to run QGC for PX4.

- **Batteries**: Some units have integrated batteries, while others have swappable ones.

- **Durability**: Some units have ruggedised housings with dust and water ratings, while others do not.

All of these factors are reflected in the price and weight of the GCS.
