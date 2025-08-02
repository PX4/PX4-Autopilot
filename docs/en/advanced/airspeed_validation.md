# Airspeed Validation

PX4 includes a set of airspeed validation checks that run continuously during fixed-wing flight to ensure the airspeed data is accurate and reliable.

If any check fails persistently, the system can declare airspeed as invalid. When this happens, PX4 discards the sensor data and falls back to the option configured by [ASPD_FALLBACK](#aspd_fallback_table).

::: info
By default, the [Missing Data](#missing-data-check), [Data Stuck](#data-stuck-check), and [Innovation](#innovation-check) checks are enabled.
You can configure which checks are active using the [ASPD_DO_CHECKS](#aspd_do_checks_table) parameter.
:::

## Validation Checks

These checks are primarily designed to validate incoming airspeed sensor data and detect potential sensor failures during flight. However, they may also trigger due to configuration issues, estimator inaccuracies, or specific flight conditions that lead to physically inconsistent measurements.

The following overview summarizes each check, common failure causes, and relevant configuration parameters.

### Missing Data Check

::: info
This check is independent of the other validation checks and cannot be disabled or configured.
It must pass for any of the other checks to run.
:::

Triggers when no new airspeed data has been received for more than 1 second.

Common failure causes:

- Faulty or disconnected sensor.

### Data Stuck Check

Triggers when the measured indicated airspeed (IAS) has not changed for more than 2 seconds.

Common failure causes:

- Faulty or blocked airspeed sensor (pitot tube).
- Very low sensor resolution.

### Innovation Check

Compares the estimated true airspeed (TAS) to the predicted GNSS-based airspeed (groundspeed minus windspeed).
If the difference exceeds a threshold ([ASPD_FS_INNOV](#aspd_fs_innov_table)) for a prolonged period ([ASPD_FS_INTEG](#aspd_fs_integ_table)), the check fails.

This check helps detect cases where the measured airspeed is inconsistent with the aircraft’s predicted kinematic state, often indicating sensor drift, (partial) blockage, or sub-optimal sensor placement on the vehicle.

Common failure causes:

- Faulty or blocked airspeed sensor (_sensor fault_)
- Poor sensor placement on vehicle (_configuration issue_)
- Poor GNSS data (_estimator innacuracy_)
- Inaccurate wind estimate (_estimator innacuracy_)

Relevant parameters: [ASPD_FS_INNOV](#aspd_fs_innov_table), [ASPD_FS_INTEG](#aspd_fs_integ_table)

### Load Factor Check

Checks whether the measured airspeed is physically consistent with the aircraft's current load factor.
If the measured airspeed is too low to plausibly generate the required lift, the sensor reading is considered invalid.

This check helps detect cases where the airspeed sensor may be under-reading during flight.

Common failure causes:

- Faulty or blocked airspeed sensor (_sensor fault_)
- Uncalibrated airspeed sensor (_configuration issue_)
- Poor sensor placement on vehicle (_configuration issue_)
- Incorrect stall speed configuration (_configuration issue_)
- The vehicle is stalling (_flight condition_)

Relevant parameters: [FW_AIRSPD_STALL](#fw_airspd_stall_table)

### First Principle Check

Validates whether the measured indicated airspeed (IAS) behavior matches expected aircraft response based on throttle and pitch inputs.

Specifically, when throttle is above trim by at least 5% and the aircraft is pitched downward (below [FW_PSP_OFF](#fw_psp_off_table)), the rate of change of the IAS should be consistent with the aircraft behaviour.

If the airspeed readings increase too slowly, this may indicate that the sensor is not responding correctly to dynamic pressure changes — for example, due to pitot tube icing or (partial) blockage.

Common failure causes:

- Faulty or blocked airspeed sensor (_sensor fault_)
- Pitot tube icing (_sensor fault_)
- Excessive drag (e.g. flaps down, landing gear deployed, payload) (_flight condition_)
- Throttle not producing expected thrust (_mechanical issue_)
- Incorrect trim or max throttle setting (_configuration issue_)

Relevant parameters: [ASPD_FP_T_WINDOW](#aspd_fp_t_window_table), [FW_PSP_OFF](#fw_psp_off_table), [FW_THR_TRIM](#fw_thr_trim_table), [FW_THR_MAX](#fw_thr_max_table)

## TAS Scale Considerations

The True Airspeed (TAS) scale is a dynamic correction factor used to account for discrepancies between measured Indicated Airspeed (IAS) and the actual airspeed an aircraft experiences in flight.
PX4 estimates this TAS scale using GNSS ground speed and wind estimation during fixed-wing flight.

This scaling plays an important role in keeping the [innovation check](#innovation-check) reliable, since a well-estimated TAS is key to spotting inconsistencies between measured and predicted airspeed.
If the TAS scale is incorrect, it can mask real airspeed faults or trigger false positives.

If the estimated scale appears consistently off, you can override it by setting a fixed value using [ASPD_SCALE_n](#aspd_scale_n_table) (where `n` is the sensor number), and disable the estimated scale with [ASPD_SCALE_APPLY](#aspd_scale_apply_table).

## Additional Configuration

To configure the delay before PX4 starts or stops using airspeed sensor data after it passes or fails validation, use:

[ASPD_FS_T_START](#aspd_fs_t_start_table): Delay after passing validation before the sensor is considered valid.

[ASPD_FS_T_STOP](#aspd_fs_t_stop_table): Delay after failing validation before the sensor is considered invalid.


## Parameters

Listed below are all the relevant paramaters.

| Parameter                                                                                                            | Description                                                                                              | Used In                                |
| -------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| <a id="aspd_do_checks_table"></a>[ASPD_DO_CHECKS](../advanced_config/parameter_reference.md#ASPD_DO_CHECKS)          | Bitmask to enable/disable individual airspeed validation checks.                                         |                                     |
| <a id="aspd_fs_innov_table"></a>[ASPD_FS_INNOV](../advanced_config/parameter_reference.md#ASPD_FS_INNOV)             | Innovation threshold between estimated and predicted TAS.                                                | [Innovation Check](#innovation-check)             |
| <a id="aspd_fs_integ_table"></a>[ASPD_FS_INTEG](../advanced_config/parameter_reference.md#ASPD_FS_INTEG)             | Threshold the integral of the innovation must exceed before triggering failure.                          | [Innovation Check](#innovation-check)             |
| <a id="fw_airspd_stall_table"></a>[FW_AIRSPD_STALL](../advanced_config/parameter_reference.md#FW_AIRSPD_STALL)       | Estimated stall speed of vehicle. If this is misconfigured, the load factor check may trigger.           | [Load Factor Check](#load-factor-check)            |
| <a id="fw_psp_off_table"></a>[FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF)                      | Pitch at level flight. If this is misconfigured, the first principle check may trigger.                  | [First Principle Check](#first-principle-check)        |
| <a id="fw_thr_trim_table"></a>[FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)                   | Throttle trim value. If this is misconfigured, the first principle check may trigger.                    | [First Principle Check](#first-principle-check)        |
| <a id="fw_thr_max_table"></a>[FW_THR_MAX](../advanced_config/parameter_reference.md#FW_THR_MAX)                      | Throttle limit max.                                                                                      | [First Principle Check](#first-principle-check)        |
| <a id="aspd_fp_t_window_table"></a>[ASPD_FP_T_WINDOW](../advanced_config/parameter_reference.md#ASPD_FP_T_WINDOW)    | Time window for evaluating airspeed trend in First Principle Check.                                      | [First Principle Check](#first-principle-check)        |
| <a id="aspd_fs_t_start_table"></a>[ASPD_FS_T_START](../advanced_config/parameter_reference.md#ASPD_FS_T_START)       | Delay after passing validation before using sensor data. Affects how soon a recovered sensor is trusted. | All Checks                |
| <a id="aspd_fs_t_stop_table"></a>[ASPD_FS_T_STOP](../advanced_config/parameter_reference.md#ASPD_FS_T_STOP)          | Delay after failure before declaring data invalid. Affects how quickly faults lead to rejection.         | All Checks                  |
| <a id="aspd_fallback_table"></a>[ASPD_FALLBACK](../advanced_config/parameter_reference.md#ASPD_FALLBACK)             | Determines fallback mode when airspeed data is lost or invalid.                                          | System behavior (post-check) |
| <a id="aspd_scale_apply_table"></a>[ASPD_SCALE_APPLY](../advanced_config/parameter_reference.md#ASPD_SCALE_APPLY)    | Controls if/when to apply estimated TAS scaling. Poor scaling can cause false innovation failures.       | [Innovation Check](#innovation-check) (indirect) |
| <a id="aspd_scale_n_table"></a>[ASPD_SCALE_n](../advanced_config/parameter_reference.md#ASPD_SCALE_1)                | User-defined IAS to TAS scale override per sensor. May help when auto-scale estimation is unreliable.    | [Innovation Check](#innovation-check) (indirect) |
