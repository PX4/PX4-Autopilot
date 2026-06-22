# Airspeed Scale Handling

:::info
This section complements the existing [Airspeed Validation](../advanced_config/airspeed_validation.md) documentation.
:::

The airspeed scale is used by PX4 to convert the measured airspeed (indicated airspeed) to the calibrated airspeed.
This scale can be set by [ASPD_SCALE_n](../advanced_config/parameter_reference.md#ASPD_SCALE_1) (where `n` is the sensor number), and logged in [AirspeedWind.msg](../msg_docs/AirspeedWind.md).

Note that the airspeed scale is different from the airspeed sensor offset calibration done on the ground at 0 m/s. The airspeed scale accounts for errors in the airspeed measurement during flight, such as those caused by sensor placement or installation effects.

This topic describes how to set an initial airspeed scale for a new fixed-wing vehicle during its first flight. Correct scale calibration ensures reliable airspeed data, accurate TAS calculation, robust PX4 airspeed validation, and consistent controller performance.

## Airspeed in PX4

PX4 handles multiple types of airspeed:

- **IAS (Indicated Airspeed):** The raw measurement from the airspeed sensor, directly influenced by sensor characteristics and installation effects (e.g., pitot-static errors).
- **CAS (Calibrated Airspeed):** IAS corrected for sensor-specific and installation-related errors.
- **EAS (Equivalent Airspeed):** _Not explicitly handled by PX4_ - Calibrated airspeed corrected for compressibility effects.
  While PX4 does not currently model EAS separately, this correction is negligible at low speeds and altitudes, so EAS is treated as equivalent to CAS for simplicity.
- **TAS (True Airspeed):** CAS adjusted for atmospheric effects such as air pressure and temperature (i.e., altitude and atmospheric conditions).

The standard conversion chain used in PX4 is: `IAS → CAS (= EAS) → TAS`.

## CAS Scale Estimation

PX4 estimates the IAS to CAS scale (referred to as the CAS scale) during flight using GNSS ground speed and wind estimation.
To compute the final TAS, standard environment conversions are applied (CAS → TAS).

:::warning
Important
A GNSS is required for scale estimation.
:::

PX4 uses a two-stage approach to robustly estimate the scale:

1. **Continuous EKF Estimation**: A wind estimator constantly compares your measured airspeed against what it expects based on ground velocity (from GNSS) and estimated wind.
   If there's a consistent bias, it adjusts the scale estimate.
   The estimated scale is logged in the `AirspeedWind.msg` as the `tas_scale_raw`.
2. **Validation**: To ensure robustness, PX4 collects airspeed and ground speed data across 12 different heading segments (every 30°).
   This averages out wind estimation errors.
   The validated scale is only updated when the new estimate demonstrably reduces the error between predicted and actual ground speeds across all headings.
   The validated scale is logged in the `AirspeedWind.msg` as the `tas_scale_validated`.

### Understanding the Scale: Physical Intuition

The CAS scale is essentially a correction factor that accounts for systematic errors in your airspeed sensor installation.

- A scale of 1.0 means your sensor reads perfectly (no correction needed)
- A scale > 1.0 (e.g., 1.1) means your sensor _under-reads_ by 10%, so measured airspeed (IAS) must be multiplied by 1.1
- A scale < 1.0 (e.g., 0.9) means your sensor _over-reads_ by ~11%, so measured airspeed (IAS) must be multiplied by 0.9

### What Affects the Airspeed Scale

The primary factor influencing the airspeed scale is **sensor placement**.

Biased readings can be reflected in the scale estimate for pitot tubes installed:

- In regions experiencing disturbed flow (commonly near blunt aircraft noses)
- Near propellers
- Under aerodynamic surfaces
- At an angle with respect to the airflow

### Symptoms of Incorrect Scale

Symptoms of an incorrectly scaled airspeed measurement include:

- Stalling or overspeeding
- Persistent under- or overestimation of the TAS relative to wind-corrected groundspeed
- False positives or missed detections in [airspeed innovation checks](../advanced_config/airspeed_validation.md#innovation-check)
- Degraded tracking of the rate controllers

## Recommended First Flight Process

During the first flight of a new fixed-wing vehicle, allocate time for the CAS scale to converge to a reasonable initial estimate.
Follow these steps:

1. **Set an Initial Scale**

   Set the CAS scale (`ASPD_SCALE_n`) to 1.0 (the default value).
   When the scale is at exactly 1.0, PX4 automatically accelerates the learning process during the first 5 minutes of flight, allowing faster convergence to the correct scale value.

2. **Perform a Flight**

   After takeoff, place the vehicle in loiter for about 15 minutes to allow the scale estimation to converge.

   ::: tip
   Flying in circles (loiter/hold mode) is important as the scale-validation algorithm requires the aircraft to pass through multiple heading segments (12 segments covering all compass directions).
   If these heading segments aren’t completed, PX4 cannot validate the estimated scale.

:::

3. **Check Scale Convergence**

   After the flight, review the estimated scale in logs.
   Verify that:

   - `tas_scale_validated` in `AirspeedWind.msg` converged during flight.
   - `true_airspeed_m_s` (TAS) in [AirspeedValidated.msg](../msg_docs/AirspeedValidated.md) is consistent with groundspeed corrected for wind.

4. **Update the Airframe Configuration**

   If using an [airframe configuration file](../dev_airframes/adding_a_new_frame.md): update `ASPD_SCALE_n`with the estimated CAS scale for future flights.
   For similar vehicles with similarly mounted sensors, this value is typically a reliable starting point.

:::info
If you are not able to perform the steps outlined above ...

For a quick manual CAS scale estimate, compare groundspeed minus windspeed (from the [VehicleLocalPosition](../msg_docs/VehicleLocalPosition.md) and [Wind](../msg_docs/Wind.md) messages, respectively) to indicated airspeed values (in the [Airspeed](../msg_docs/Airspeed.md) message).
The ratio of indicated airspeed to groundspeed minus windspeed can provide a reasonable starting estimate for `ASPD_SCALE_n`.
:::
