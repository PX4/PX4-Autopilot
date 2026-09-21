# PX4 Flight Stack — C4 Model

A [C4 model](https://c4model.com/) (Context → Containers → Components → Dynamic) of this Sees fork of
PX4-Autopilot: what the flight stack talks to, what actually gets deployed, how the ~50 runtime modules
divide responsibility, and how that maps onto `src/` on disk.

Rendered version (with light/dark theming and a section nav): [PX4 Flight Stack artifact](https://claude.ai/artifact/8MDFmfvCbTmD6xeyJxAHLo).

Generated from a static read of `vilas/fix/v1.17` — directory listing, module `PRINT_MODULE_DESCRIPTION` text,
and `SEES_V1.17_MIGRATION.md`. Re-derive after any module reshuffle.

## Legend

| Element | Meaning |
|---|---|
| **Person** | A human interacting with the system |
| **System in scope** | The PX4 Flight Stack itself |
| **Container** | An independently deployable/runnable unit |
| **Component** | A module/task running inside a container |
| **External system** | Hardware or software this repo doesn't own |

---

## 1. System Context

The flight stack sits between a human pilot/operator and the physical world it flies through. Everything
outside the flight stack box is hardware or software this repo does not own, but must speak a protocol to —
MAVLink, uORB-over-DDS, PWM/DShot, or a bus like I2C/SPI/CAN.

```mermaid
flowchart TB
  classDef person fill:#6a5a8c,stroke:#443861,color:#ffffff
  classDef system fill:#c9821f,stroke:#8a5814,color:#211100
  classDef external fill:#6b7280,stroke:#464b52,color:#ffffff

  pilot["RC / Safety Pilot\n(person)\nManual stick input,\nkill-switch override"]:::person
  operator["Mission / Payload Operator\n(person)\nPlans missions, monitors\ntelemetry via QGC"]:::person

  px4["PX4 Flight Stack\n[software system]\nReal-time state estimation,\nflight control & autonomy"]:::system

  gcs["QGroundControl\n[external system]\nGround control station\nover MAVLink"]:::external
  companion["Companion Computer / ROS 2\n[external system]\nOffboard compute over\nuXRCE-DDS / Zenoh / MAVLink"]:::external
  sensorsuite["Sensor Suite\n[external hardware]\nIMU, GNSS, baro, mag,\nairspeed, IRLock, camera"]:::external
  actuators["ESCs, Servos, Motors\n[external hardware]\nPWM / DShot / DroneCAN"]:::external
  pio["PX4IO Co-processor\n[external hardware]\nSeparate I/O MCU (SPI)"]:::external
  storage["microSD Card\n[external hardware]\nLogs, params, missions"]:::external
  safety["Kill-switch, Buzzer,\nSafety Button\n[external hardware]"]:::external
  remoteid["Remote-ID / ADS-B\nTransponder\n[external hardware]"]:::external

  pilot -->|"RC sticks & switches\n(PPM/SBUS/CRSF radio)"| px4
  operator -->|"missions, joystick,\ntelemetry review"| gcs
  gcs -->|MAVLink| px4
  px4 -->|"telemetry, status"| gcs
  companion -->|"offboard setpoints,\nROS 2 topics"| px4
  px4 -->|"uORB topics over DDS/Zenoh"| companion
  sensorsuite -->|"raw measurements\n(I2C/SPI/UART/CAN)"| px4
  px4 -->|"actuator commands"| actuators
  px4 -->|"PWM mix, RC passthrough cfg"| pio
  pio -->|"RC input, output status"| px4
  px4 -->|"logs, params, missions"| storage
  safety -->|"arm inhibit / kill signal"| px4
  px4 -->|"arm & kill tone, LED"| safety
  px4 -->|"position / ID broadcast"| remoteid
```

> QGroundControl is both a command source (missions, parameters, joystick) and a telemetry sink over the
> same MAVLink link.

---

## 2. Containers

Unlike a typical web system, "PX4 Flight Stack" is **not** a fleet of independently deployed services — on a
real airframe it is almost entirely one statically-linked binary. The honest container boundaries are: the
main firmware image, a genuinely separate co-processor firmware (PX4IO), the bootloader that flashes both,
and an alternate build target (SITL) used for desktop simulation instead of real hardware.

```mermaid
flowchart TB
  classDef container fill:#2f6f76,stroke:#1c4a4f,color:#ffffff
  classDef external fill:#6b7280,stroke:#464b52,color:#ffffff

  subgraph boundary["PX4 Flight Stack — system boundary"]
    direction TB
    fw["Flight Stack Firmware\n[NuttX RTOS binary, C/C++]\nAll flight-critical modules:\nestimation, control, nav, comms"]:::container
    io_fw["PX4IO Firmware\n[NuttX, separate MCU]\nRC mixing, PWM/DShot output,\nsafety-button & IO failsafe"]:::container
    boot["Bootloader\n[protected flash region]\nUSB/CAN entry point for\nflashing fw & io_fw"]:::container
    romfs["ROMFS init scripts\n[baked into firmware image]\nBoard/airframe start-up,\nmixer & parameter defaults"]:::container
  end

  sitl["SITL Build\n[POSIX process]\nSame module source tree,\nsimulated sensors & clock"]:::container

  gcs["QGroundControl"]:::external
  sim["Gazebo / jMAVSim"]:::external
  companion["Companion Computer / ROS 2"]:::external
  sensorsuite["Sensor Suite"]:::external
  actuators["ESCs / Servos"]:::external

  romfs -->|"loaded at boot"| fw
  boot -->|flashes| fw
  boot -->|flashes| io_fw
  fw -->|"SPI protocol: mixed\noutputs out, RC in"| io_fw
  fw -->|MAVLink/DDS/Zenoh| gcs
  fw -->|MAVLink/DDS/Zenoh| companion
  fw -->|"actuator commands"| actuators
  sensorsuite -->|"raw measurements"| fw
  sitl -->|MAVLink| gcs
  sitl -->|"simulated actuators"| sim
  sim -->|"simulated sensors"| sitl
```

A given `boards/<vendor>/<model>` target decides which drivers and modules get linked into the "Flight Stack
Firmware" container — see [Code layout](#4-code-layout).

---

## 3. Components — inside "Flight Stack Firmware"

Everything below runs as its own scheduled task (or work-queue job) inside the one firmware binary. **They
never call each other directly** — every arrow is a publish/subscribe relationship over the `uORB` message
bus (`platforms/common/uORB`, schemas in `msg/*.msg`). Arrow direction shows data flow, not a function call.

```mermaid
flowchart LR
  classDef component fill:#3d5a80,stroke:#25384f,color:#ffffff

  subgraph L1[Drivers]
    drv_sensors["Sensor Drivers\nimu · gps/gnss · barometer ·\nmagnetometer · airspeed ·\ndistance_sensor · irlock · camera"]:::component
    drv_rc["RC Input Driver\nrc_input · px4io · crsf"]:::component
    drv_act["Actuator Output Drivers\npwm_out · dshot · uavcan ·\nlinux_pwm_out · pca9685"]:::component
  end

  subgraph L2[Estimation & Perception]
    sens["sensors\nvoting, calibration,\nfusion hub"]:::component
    ekf2["ekf2\nattitude/position EKF"]:::component
    estx["mag_bias_estimator ·\nwind_estimator ·\nhover_thrust_estimator"]:::component
    lte["landing_target_estimator\n(IRLock precision landing)"]:::component
    lnd["land_detector\nlanded / freefall state"]:::component
  end

  subgraph L3[Command · Input · Autonomy]
    rcu["rc_update\nRC mapping & calibration"]:::component
    manctl["manual_control\nRC/MAVLink joystick\narbitration"]:::component
    cmd["commander\narming, failsafe,\nhealth checks, mode FSM"]:::component
    nav["navigator\nmissions, geofence, RTL\n(reads dataman)"]:::component
    fmm["flight_mode_manager /\nfw_mode_manager\nper-mode setpoint gen"]:::component
  end

  subgraph L4[Control Chain]
    mc["mc_pos_control →\nmc_att_control →\nmc_rate_control"]:::component
    fwc["fw_lateral_longitudinal_control\n→ fw_rate_control"]:::component
    rover["rover_ackermann /\ndifferential / mecanum"]:::component
    vtol["vtol_att_control"]:::component
    alloc["control_allocator\ntorque/thrust →\nactuator setpoints"]:::component
  end

  subgraph L5[Comms & Storage]
    mav["mavlink\nGCS/companion link,\nmission protocol"]:::component
    dds["uxrce_dds_client\nROS 2 / DDS bridge"]:::component
    zen["zenoh\nlow-latency pub/sub\nbridge"]:::component
    log["logger\nULog flight recorder"]:::component
    dm["dataman\nmission/geofence\nstorage"]:::component
  end

  drv_sensors --> sens
  drv_rc --> rcu
  sens --> ekf2
  sens --> estx
  estx -.-> ekf2
  ekf2 --> lnd
  ekf2 --> nav
  ekf2 --> fmm
  lte -.-> fmm
  rcu --> manctl
  manctl --> cmd
  manctl --> fmm
  cmd --> nav
  cmd --> fmm
  nav --> fmm
  nav <--> dm
  fmm --> mc
  fmm --> fwc
  fmm --> rover
  fmm --> vtol
  mc --> alloc
  fwc --> alloc
  rover --> alloc
  vtol --> alloc
  alloc --> drv_act
  mav <--> cmd
  mav <--> nav
  dds -.-> cmd
  zen -.-> sens
  ekf2 -.-> log
  cmd -.-> log
  mav -.-> log
```

Dashed arrows mark lighter/occasional relationships (diagnostic bridges, log sampling, optional inputs)
versus the solid control-and-estimation backbone. In reality *every* node above also holds a direct
subscribe/advertise handle to uORB itself — that fan-out is omitted here for legibility; the arrows shown
are the meaningful data-flow edges.

---

## Dynamic view — multicopter position-hold loop

One concrete run through the Level 3 diagram: a stick input turns into motor commands. This is the
highest-rate path in the system, re-run at up to 500 Hz for the inner rate loop.

```mermaid
sequenceDiagram
  participant S as Sensor Drivers
  participant SH as sensors
  participant EKF as ekf2
  participant FMM as flight_mode_manager
  participant PC as mc_pos_control
  participant AC as mc_att_control
  participant RC as mc_rate_control
  participant CA as control_allocator
  participant AD as Actuator Drivers

  S->>SH: raw IMU / GNSS / baro / mag samples
  SH->>EKF: sensor_combined, vehicle_gps_position
  EKF->>FMM: vehicle_local_position, vehicle_attitude
  FMM->>PC: vehicle_control_mode, trajectory_setpoint
  PC->>AC: vehicle_attitude_setpoint (+ thrust)
  AC->>RC: vehicle_rates_setpoint
  RC->>CA: vehicle_torque_setpoint, vehicle_thrust_setpoint
  CA->>AD: actuator_motors / actuator_servos
  Note over S,AD: every arrow is a uORB publish → subscribe,<br/>never a direct function call between modules
```

---

## 4. Code layout

How the directories in this repository correspond to the elements drawn above.

| Path | Contains | Maps to |
|---|---|---|
| `src/drivers/` | ~70 peripheral driver families (imu, gnss, pwm_out, dshot, uavcan, irlock, tone_alarm, safety_button…) | "Drivers" swimlane, Level 3 |
| `src/modules/` | ~50 application modules — one directory per runtime task (commander, navigator, ekf2, mavlink, logger…) | Most Level 3 components |
| `src/lib/` | Shared libraries linked into modules: `matrix`, `geo`, `mixer_module`, `control_allocation`, `rate_control`, `tecs`, `npfg`, `rtl`, `battery`, `sensor_calibration`… | Implementation detail inside components, not separate C4 elements |
| `src/systemcmds/` | Shell utilities (`uorb`, `param`, `reboot`, `mtd`…) | Operational tooling around Platform Core |
| `platforms/{nuttx,posix,qurt,common,ros2}/` | RTOS/POSIX/QuRT glue, the uORB pub-sub implementation, work queues, HRT scheduler | Platform Core + selects Firmware vs. SITL container |
| `boards/<vendor>/<model>/` | Per-board CMake config: which drivers/modules compile in, default mixer & pinout | Chooses the concrete component set for one Firmware container instance |
| `msg/` (245 files) | uORB topic schemas — the interface contract between every producer and consumer | The wires in the Level 3 diagram |
| `ROMFS/` | Startup shell scripts (`rcS`, airframe & mixer configs) baked into the firmware image | "ROMFS init scripts" container |
| `src/modules/px4iofirmware/` | Firmware for the separate RC-input/PWM-output co-processor MCU | "PX4IO Firmware" container |
| `Tools/` | Build & codegen (uORB message generation), CI, simulation helpers | Build-time tooling — not present at runtime |

---

## 5. Fork overlay — where the Sees customizations land

This is not stock PX4 — it's PX4 v1.17.0-dev with Sees-specific features layered on top (per
`SEES_V1.17_MIGRATION.md`). None of these change the model above; they extend specific components.

| Feature | Touches |
|---|---|
| Safety-pilot RC/Mavlink control-source selector | `rc_update`, `manual_control`, `commander`, telemetry streams in `mavlink` |
| Battery SOC / chemistry curve tuning | `src/lib/battery` |
| Dual-CAN-GPS node-ID ordering | `src/drivers/gnss`, UAVCAN/Cyphal channel assignment |
| Custom magnetometer filtering + noise metric | `sensors` module, `src/drivers/magnetometer` |
| FrSky/Horus telemetry customization | `mavlink` (Smartport DIY streams) |
| Landing-target-estimator auto-start on `SENS_EN_IRLOCK` | `landing_target_estimator`, `commander` start-up logic |
| Default-enabled IRLock / landing-target-pose logging | `logger` |
| STATUSTEXT broadcast to onboard controller without GCS | `mavlink` |
| Extra MAVLink instance | `mavlink` start-up config (`ROMFS`) |
| Audible tone on kill-switch / AUTO_RTL entry (BVLOS compliance) | `commander`, `src/drivers/tone_alarm` |
