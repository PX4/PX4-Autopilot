# Safety (Failsafe) Configuration

PX4 має кілька функцій безпеки для захисту та відновлення вашого транспортного засобу в разі виникнення проблем:

- _Failsafes_ allow you to specify areas and conditions under which you can safely fly, and the [action](#failsafe-actions) that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).
  The most important failsafe settings are configured in the _QGroundControl_ [Safety Setup](#qgroundcontrol-safety-setup) page.
  Others must be configured via [parameters](../advanced_config/parameters.md).
- [Safety switches](#emergency-switches) on the remote control can be used to immediately stop motors or return the vehicle in the event of a problem.

## Налаштування безпеки QGroundControl

The _QGroundControl_ Safety Setup page is accessed by clicking the _QGroundControl_ icon, **Vehicle Setup**, and then **Safety** in the sidebar.
This includes many of the most important failsafe settings (battery, RC loss etc.) and the settings for the triggered actions _Return_ and _Land_.

![Safety Setup(QGC)](../../assets/qgc/setup/safety/safety_setup.png)

## Дії аварійного режиму

When a failsafe is triggered, the default behavior (for most failsafes) is to enter Hold for [COM_FAIL_ACT_T](../advanced_config/parameter_reference.md#COM_FAIL_ACT_T) seconds before performing an associated failsafe action.Це дає користувачу час помітити, що відбувається, і перевизначити аварійний режим, якщо це необхідно.У більшості випадків це можна зробити, використовуючи RC або GCS для перемикання режимів (зверніть увагу, що під час утримання аварійного режиму переміщення пультів RC не спричиняє перезапуску).

Нижче наведений список всіх дій аварійного режиму, впорядкованих за зростанням серйозності.
Зверніть увагу, що різні типи аварійного режиму можуть не підтримувати всі ці дії.

| Дія                                                    | Опис                                                                                                                                                                                                                                                                                                                                                |
| ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="act_none"></a>None/Disabled                     | Немає дій Система аварійного відключення буде ігноруватися.                                                                                                                                                                                                                                                                         |
| <a id="act_warn"></a>Warning                           | A warning message will be sent (i.e. to _QGroundControl_).                                                                                                                                                                                                                       |
| <a id="act_hold"></a>Hold mode                         | The vehicle will enter [Hold mode (MC)](../flight_modes_mc/hold.md) or [Hold mode (FW)](../flight_modes_fw/hold.md) and hover or circle, respectively. Літальні апарати VTOL будуть утримуватися відповідно до їх поточного режиму (MC/FW).                |
| <a id="act_return"></a>[Return mode][return]           | The vehicle will enter _Return mode_. Return behaviour can be set in the [Return Home Settings](#return-mode-settings) (below).                                                                                                                                                                  |
| <a id="act_land"></a>Land mode                         | The vehicle will enter [Land mode (MC)](../flight_modes_mc/land.md) or [Land mode (FW)](../flight_modes_fw/land.md), and land. Спочатку VTOL перейде в режим MC.                                                                                                              |
| <a id="act_disarm"></a>Disarm                          | Зупиняє мотори негайно.                                                                                                                                                                                                                                                                                                             |
| <a id="act_term"></a>[Flight termination][flight_term] | Turns off all controllers and sets all PWM outputs to their failsafe values (e.g. [PWM\_MAIN\_FAILn][pwm_main_failn], [PWM\_AUX\_FAILn][pwm_main_failn]). Захисні виходи можуть бути використані для розгортання парашута, шасі або виконання іншої операції. Для літального апарату це може дозволити вам плавно спустити апарат у безпечне місце. |

[flight_term]: ../advanced_config/flight_termination.md
[return]: ../flight_modes/return.md
[pwm_main_failn]: ../advanced_config/parameter_reference.md#PWM_MAIN_FAIL1
[pwm_aux_failn]: ../advanced_config/parameter_reference.md#PWM_AUX_FAIL1

Якщо спрацьовують декілька запобіжників, вживається більш сувора дія.
For example if both RC and GPS are lost, and manual control loss is set to [Return mode](#act_return) and GCS link loss to [Land](#act_land), Land is executed.

:::tip
The exact behavior when different failsafes are triggered can be tested with the [Failsafe State Machine Simulation](safety_simulation.md).
:::

### Налаштування режиму повернення

<!-- Propose replace section by a summary and links - return mode is complicated -->

_Return_ is a common [failsafe action](#failsafe-actions) that engages [Return mode](../flight_modes/return.md) to return the vehicle to the home position.
The default settings for each vehicle are usually suitable, though for fixed wing vehicles you will usually need to define a mission landing.

:::tip
If you want to change the configuration you should carefully read the [Return mode](../flight_modes/return.md) documentation _for your vehicle type_ to understand the options.
:::

QGC allows users to set some aspects of the return mode and landing behaviour, such as the altitude to fly back, and the loiter time if you need to deploy landing gear.

![Safety - Return Home Settings (QGC)](../../assets/qgc/setup/safety/safety_return_home.png)

### Налаштування режиму землі

_Land at the current position_ is a common [failsafe action](#failsafe-actions) (in particular for multicopters), that engages [Land Mode](../flight_modes_mc/land.md).
The default settings for each vehicle are usually suitable.

:::tip
If you want to change the configuration you should carefully read the [Land mode](../flight_modes_fw/land.md) documentation _for your vehicle type_ to understand the options.
:::

QGC allows users to set some aspects of the landing behaviour, such as the time to disarm after landing and the descent rate (for multicopters only).

![Safety - Land Mode Settings (QGC)](../../assets/qgc/setup/safety/safety_land_mode.png)

## Battery Failsafes

### Battery level failsafe

The low battery failsafe is triggered when the battery capacity drops below battery failafe level values.
You can configure both the levels and the failsafe actions at each level in QGroundControl.

![Safety - Battery (QGC)](../../assets/qgc/setup/safety/safety_battery.png)

The most common configuration is to set the values and action as above (with `Warn > Failsafe > Emergency`), and to set the [Failsafe Action](#COM_LOW_BAT_ACT) to warn at "warn level", trigger Return mode at "Failsafe level", and land immediately at "Emergency level".

Налаштування та вибрані параметри показані нижче.

| Налаштування                                        | Parameter                                                                                                                                   | Опис                                                                                                             |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| <a id="COM_LOW_BAT_ACT"></a>Failsafe Action         | [COM_LOW_BAT_ACT](../advanced_config/parameter_reference.md#COM_LOW_BAT_ACT) | Warn, Return, or Land based when capacity drops below the trigger levels.                        |
| <a id="BAT_LOW_THR"></a>Battery Warn Level          | [BAT_LOW_THR](../advanced_config/parameter_reference.md#BAT_LOW_THR)                              | Відсоткова ємність для попереджень (або інших дій).                           |
| <a id="BAT_CRIT_THR"></a>Battery Failsafe Level     | [BAT_CRIT_THR](../advanced_config/parameter_reference.md#BAT_CRIT_THR)                            | Відсоткова ємність для дії Повернення (або інших дій, якщо вибрано одну дію). |
| <a id="BAT_EMERGEN_THR"></a>Battery Emergency Level | [BAT_EMERGEN_THR](../advanced_config/parameter_reference.md#BAT_EMERGEN_THR)                      | Відсоткова ємність для спрацьовування дії Land (негайно).                     |

### Flight Time Failsafes

There are several other "battery related" failsafe mechanisms that may be configured using parameters:

- The "remaining flight time for safe return" failsafe ([COM_FLTT_LOW_ACT](#COM_FLTT_LOW_ACT)) is engaged when PX4 estimates that the vehicle has just enough battery remaining for a return mode landing.
  You can configure this to ignore the failsafe, warn, or engage Return mode.
- The "maximum flight time failsafe" ([COM_FLT_TIME_MAX](#COM_FLT_TIME_MAX)) allows you to set a maximum flight time after takeoff, at which the vehicle will automatically enter return mode (it will also "warn" at 90% of this time). This is like a "hard coded" estimate of the total flight time in a battery. The feature is disabled by default.
- The "minimum battery" for arming parameter ([COM_ARM_BAT_MIN](#COM_ARM_BAT_MIN)) prevents arming in the first place if the battery level is below the specified value.

Налаштування та вибрані параметри показані нижче.

| Налаштування                                                         | Parameter                                                                                                                                     | Опис                                                                                                                                                                                                                               |
| -------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_FLTT_LOW_ACT"></a> Low flight time for safe return action | [COM_FLTT_LOW_ACT](../advanced_config/parameter_reference.md#COM_FLTT_LOW_ACT) | Action when return mode can only just reach safety with remaining battery. `0`: None (default), `1`: Warning, `3`: Return mode. |
| <a id="COM_FLT_TIME_MAX"></a> Maximum flight time failsafe level     | [COM_FLT_TIME_MAX](../advanced_config/parameter_reference.md#COM_FLT_TIME_MAX) | Maximum allowed flight time before Return mode will be engaged, in seconds. `-1`: Disabled (default).                                                           |

## Manual Control Loss Failsafe

A [Manual Control Loss Failsafe](../config/safety.md#manual-control-loss-failsafe) is triggered after a [manual control loss timeout](#COM_RC_LOSS_T) in which none of the configured [Manual Controllers](../config/manual_control.md) are available.

![Safety - RC Loss (QGC)](../../assets/qgc/setup/safety/safety_rc_loss.png)

The QGCroundControl Safety UI allows you to set the [failsafe action](#failsafe-actions) and [manual control loss timeout](#COM_RC_LOSS_T).
Users that want to disable this failsafe in specific modes can do so using the parameter [COM_RCL_EXCEPT](#COM_RCL_EXCEPT).

:::info
PX4 and the receiver may also need to be configured in order to _detect RC loss_: [Radio Setup > RC Loss Detection](../config/radio.md#rc-loss-detection).
:::

Нижче наведено додаткові (і базові) налаштування параметрів.

| Parameter                                                                                                                                                            | Налаштування                                     | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_RC_LOSS_T"></a>[COM_RC_LOSS_T](../advanced_config/parameter_reference.md#COM_RC_LOSS_T)    | Аварійний режим втрати ручного керування Timeout | Час після отримання останньої встановленої точки від вибраного джерела керування вручну, після якого керування вважається втраченим. This must be kept short because the vehicle will continue to fly using the last known stick position until the timeout triggers.                                                                                                                                                                                                                                                    |
| <a id="COM_FAIL_ACT_T"></a>[COM_FAIL_ACT_T](../advanced_config/parameter_reference.md#COM_FAIL_ACT_T) | Затримка відмови від дії                         | Delay in seconds between failsafe condition being triggered (`COM_RC_LOSS_T`) and failsafe action (RTL, Land, Hold). У цьому стані транспортний засіб очікує в режимі утримання на повторне підключення джерела керування вручну. Це може бути встановлено довше для довгих польотів, щоб втрата інтермітентного з'єднання не викликала негайного виклику аварійного режиму. Це може бути рівним нулю, щоб аварійний запобіжник спрацював негайно. |
| <a id="NAV_RCL_ACT"></a>[NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT)                               | Моделювання відмовостійкості                     | Disabled, Loiter, Return, Land, Disarm, Terminate, Hold mode (no failsafe).                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| <a id="COM_RCL_EXCEPT"></a>[COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT)                      | Виключення втрат RC                              | Set modes in which manual control loss is ignored.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

### Hold Mode (No Failsafe)

The `Hold mode (no failsafe)` action (`NAV_RCL_ACT = 7`) is a special case that does _not_ enter the failsafe.
Instead, if manual control is lost while actively flying a manual mode (such as Position, Altitude, or Stabilized), the vehicle switches to [Hold mode](../flight_modes_mc/hold.md) as if the user had commanded it: there is no failsafe state and no alarming failsafe notification.
This is intended for operations where a switch to Hold on manual control loss is expected and should not be surfaced as a failsafe (for example when operating multiple drones with one GCS).

If Hold cannot be entered (for example without a valid position estimate), the normal failsafe takes over and escalates from there (Return, Land, Descend, or Terminate as applicable).
Manual control loss in any non-manual (auto/offboard) mode is ignored with this setting.

## Втрата каналу зв'язку Failsafe

The Data Link Loss failsafe is triggered if the connection to the last MAVLink ground station like QGroundControl is lost.
Users that want to disable this failsafe in specific modes can do so using the parameter [COM_DLL_EXCEPT](#COM_DLL_EXCEPT).

![Safety - Data Link Loss (QGC)](../../assets/qgc/setup/safety/safety_data_link_loss.png)

Налаштування та вибрані параметри показані нижче.

| Налаштування                                                | Parameter                                                                                                                               | Опис                                                                                  |
| ----------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| Тайм-аут втрати каналу зв'язку                              | [COM_DL_LOSS_T](../advanced_config/parameter_reference.md#COM_DL_LOSS_T) | Час після втрати з'єднання з даними перед тим, як спрацює запобіжник. |
| Моделювання відмовостійкості                                | [NAV_DLL_ACT](../advanced_config/parameter_reference.md#NAV_DLL_ACT)                          | Вимкнути, Hold mode, Return mode, Land mode, Роззброїти, Завершити.   |
| <a id="COM_DLL_EXCEPT"></a>Mode exceptions for DLL failsafe | [COM_DLL_EXCEPT](../advanced_config/parameter_reference.md#COM_DLL_EXCEPT)                    | Set modes in which data link loss is ignored.                         |

## Аварійний режим "обмеження зони політів"

The _Geofence Failsafe_ is triggered when the drone breaches a "virtual" perimeter.
У найпростішій формі периметр налаштовується як циліндр, центрований навколо домашньої позиції.
If the vehicle moves outside the radius or above the altitude the specified _Failsafe Action_ will trigger.

Note that the failsafe action will only trigger once the vehicle has already breached the geofence.
If you have a strict no-fly zone for safety or legal reasons, set [GF_ACTION](../advanced_config/parameter_reference.md#GF_ACTION) to `Hold` and include a safety margin in your geofences.
The margin should be at least:

- **Fixed-Wing**: The loiter radius [NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD).
- **Multicopter**: The stopping distance (`v^2 / 2a` with `v`=[MPC_XY_VEL_MAX](../advanced_config/parameter_reference.md#MPC_XY_VEL_MAX) and `a`=[MPC_ACC_HOR_MAX](../advanced_config/parameter_reference.md#MPC_ACC_HOR_MAX)).

Use a margin above those nominal values to account for possible tailwind, position uncertainty, attitude tracking delay, etc.

![Safety - Geofence (QGC)](../../assets/qgc/setup/safety/safety_geofence.png)

:::tip
PX4 separately supports more complicated Geofence geometries with multiple arbitrary polygonal and circular inclusion and exclusion areas: [Flying > Geofence](../flying/geofence.md).
:::

The settings and underlying [geofence parameters](../advanced_config/parameter_reference.md#geofence) are shown below.

| Налаштування                 | Parameter                                                                                                                                   | Опис                                                                                              |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| Дії у випадку порушення      | [GF_ACTION](../advanced_config/parameter_reference.md#GF_ACTION)                                                       | None, Попередження, Режим утримання, Режим повернення, Припинити, Посадка.        |
| Максимальний радіус          | [GF_MAX_HOR_DIST](../advanced_config/parameter_reference.md#GF_MAX_HOR_DIST) | Горизонтальний радіус циліндра геозони. Геозона вимкнена, якщо 0. |
| Макс. висота | [GF_MAX_VER_DIST](../advanced_config/parameter_reference.md#GF_MAX_VER_DIST) | Висота циліндра геозони. Геозона вимкнена, якщо 0.                |

:::info
Setting `GF_ACTION` to terminate will kill the vehicle on violation of the fence.
Due to the inherent danger of this, this function is disabled using [CBRK_FLIGHTTERM](#CBRK_FLIGHTTERM), which needs to be reset to 0 to really shut down the system.
:::

Також застосовуються наступні налаштування, але вони не відображаються в інтерфейсі QGC.

| Налаштування                                                       | Parameter                                                                                         | Опис                                                                                                        |
| ------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| <a id="GF_SOURCE"></a>Geofence source                              | [GF_SOURCE](../advanced_config/parameter_reference.md#GF_SOURCE)             | Встановіть, чи джерело позиції є оціненою глобальною позицією або прямо з пристрою GPS.     |
| <a id="CBRK_FLIGHTTERM"></a>Circuit breaker for flight termination | [CBRK_FLIGHTTERM](../advanced_config/parameter_reference.md#CBRK_FLIGHTTERM) | Увімкнення/вимкнення дії припинення польоту (за замовчуванням вимкнено). |

## Position Estimation Failsafes

This section describes failsafes related to the quality of the vehicle's position estimate.

### Position Loss Failsafe

The _Position Loss Failsafe_ is triggered if the quality of the PX4 position estimate falls below acceptable levels (this might be caused by GPS loss) while in a mode that requires an acceptable position estimate.

### Position Loss Failsafe Trigger

The position loss failsafe triggers if the position estimate becomes _invalid_. There are two mechanisms in PX4 to invalidate the position estimate:

- A timeout since the last sensor data was fused that provides direct speed or horizontal position measurements.
  - Sensors that fall into that category are: GNSS, optical flow, airspeed, VIO, auxiliary global position.
- The estimated horizontal position inaccuracy exceeds the threshold [COM_POS_LOW_EPH](../advanced_config/parameter_reference.md#COM_POS_LOW_EPH)
  - This check is only done on hovering systems (rotary-wing vehicles or VTOLs in hover phase). For fixed-wing vehicles, refer to the [Position Accuracy Low](#position-accuracy-low-failsafe) section.

The relevant parameters shown below.

| Parameter                                                                                                                                                            | Опис                                                                                                                                                                                                                           |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="EKF2_NOAID_TOUT"></a>[EKF2_NOAID_TOUT](../advanced_config/parameter_reference.md#EKF2_NOAID_TOUT)                   | Maximum inertial dead-reckoning time, so the time after the last data sample was received of any sensor that constrains the velocity drift [microseconds]. |
| <a id="COM_POS_FS_EPH"></a>[COM_POS_FS_EPH](../advanced_config/parameter_reference.md#COM_POS_FS_EPH) | Horizontal position error threshold for hovering vehicles (Multicopters and VTOLs in hover). Fixed-wing vehicles have this value set to infinity.                           |

### Position Loss Failsafe Action

Multicopters will switch to [Altitude mode](../flight_modes_mc/altitude.md) if a height estimate is available, otherwise [Stabilized mode](../flight_modes_mc/manual_stabilized.md).

Fixed-wing planes, and VTOLs not configured to land in hover ([NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT)), have a parameter ([FW_GPSF_LT](../advanced_config/parameter_reference.md#FW_GPSF_LT)) that defines how long they will loiter (circle with a constant roll angle ([FW_GPSF_R](../advanced_config/parameter_reference.md#FW_GPSF_R)) at the current altitude) after losing position before attempting to land.
If VTOLs have are configured to switch to hover for landing ([NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT)) then they will first transition and then descend.

Відповідними параметрами є:

| Parameter                                                                                                                                 | Опис                                                                                                                                                                                                                              |
| ----------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_GPSF_LT"></a>[FW_GPSF_LT](../advanced_config/parameter_reference.md#FW_GPSF_LT)       | Fixed-wing only: Loiter time (waiting at current altitude for position estimation recovery before starting to descend). Установіть значення 0 для відключення. |
| <a id="FW_GPSF_R"></a>[FW_GPSF_R](../advanced_config/parameter_reference.md#FW_GPSF_R)          | Фіксований кут крену/кочення під час кілочення.                                                                                                                                                                   |
| <a id="NAV_FORCE_VT"></a>[NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT) | If true, force VTOL takeoff and landing, even in `Descend` failsafe.                                                                                                                                              |

### Position Accuracy Low Failsafe

In Fixed-wing, the position estimate is never strictly invalidated as long as we have a horizontal aiding source, such as an airspeed sensor. In that case, a separate failsafe can be configured that triggers if the position estimate inacuraccy exceeds the threshold [COM_POS_LOW_EPH](../advanced_config/parameter_reference.md#COM_POS_LOW_EPH). The failsafe action is taken if the vehicle is in mission or hold mode, otherwise it is only a warning. Відповідними параметрами є:

| Parameter                                                                                                                                                               | Опис                                                                                                                                               |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_POS_LOW_EPH"></a>[COM_POS_LOW_EPH](../advanced_config/parameter_reference.md#COM_POS_LOW_EPH) | Position inaccuracy threshold above which COM_POS_LOW_ACT is taken. |
| <a id="COM_POS_LOW_ACT"></a>[COM_POS_LOW_ACT](../advanced_config/parameter_reference.md#COM_POS_LOW_ACT) | Failsafe action taken when position inaccuracy is above configured threshold.                                                      |

Note that if there is no horizontal aiding source anymore, the position estimate is invalidated after `EKF2_NOAID_TOUT`, and the standard position loss failsafe applies.

### GNSS Check Failsafe

<Badge type="tip" text="PX4 v1.18" />

Triggers on either of:

- **Count drop**: receivers with a 3D fix drop below [SYS_HAS_NUM_GNSS](#SYS_HAS_NUM_GNSS). No failsafe action when `SYS_HAS_NUM_GNSS=0` (default).
- **Position divergence**: two receivers disagree beyond their expected separation (configured via [SENS_GPS0_OFFX/Y](../advanced_config/parameter_reference.md#SENS_GPS0_OFFX), [SENS_GPS1_OFFX/Y](../advanced_config/parameter_reference.md#SENS_GPS1_OFFX)) plus reported accuracy. Only triggers a failsafe action if `SYS_HAS_NUM_GNSS=2`.

At least a warning is emitted, additional failsafe actions can be configured using [COM_GNSSLOSS_ACT](#COM_GNSSLOSS_ACT).
Loss of a single GPS when none are required is handled by other GPS health checks.

| Parameter                                                                                                                                                                  | Опис                                                                                                                                                      |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="SYS_HAS_NUM_GNSS"></a>[SYS_HAS_NUM_GNSS](../advanced_config/parameter_reference.md#SYS_HAS_NUM_GNSS) | Number of usable GNSS receivers required for arming and flight. If two are required then they also need to be consistent. |
| <a id="COM_GNSSLOSS_ACT"></a>[COM_GNSSLOSS_ACT](../advanced_config/parameter_reference.md#COM_GNSSLOSS_ACT)                      | Failsafe action when a GNSS failure is detected. Actions other than a warning also lead to arming being blocked.          |

## Аварійний режим втрати управління з пульта

The _Offboard Loss Failsafe_ is triggered if the offboard link is lost while under [Offboard control](../flight_modes/offboard.md).

Відповідні параметри наведено нижче:

| Parameter                                                                                                                                 | Опис                                                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| [COM_OF_LOSS_T](../advanced_config/parameter_reference.md#COM_OF_LOSS_T)   | Затримка після втрати зовнішнього з'єднання перед спрацюванням запобіжника. |
| [COM_OBL_RC_ACT](../advanced_config/parameter_reference.md#COM_OBL_RC_ACT) | Flight mode to switch to if offboard control is lost.                       |

## Аварійний режим уникнення трафіку

The Traffic Avoidance Failsafe allows PX4 to respond to [cooperative traffic reports](../peripherals/adsb_flarm.md).
The action parameters depend on the conflict model selected when the firmware is built.

Відповідні параметри наведено нижче:

| Parameter                                                                                                                                     | Опис                                                         |
| --------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| [DAA_EN](../advanced_config/parameter_reference.md#DAA_EN)                                                               | Enables or disables Detect and Avoid.        |
| [NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID)                        | Action for the Crosstrack model.             |
| [DAA_LVL_LOW_ACT](../advanced_config/parameter_reference.md#DAA_LVL_LOW_ACT)   | F3442-mode action for a `LOW` conflict.      |
| [DAA_LVL_MED_ACT](../advanced_config/parameter_reference.md#DAA_LVL_MED_ACT)   | F3442-mode action for a `MEDIUM` conflict.   |
| [DAA_LVL_HIGH_ACT](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_ACT) | F3442-mode action for a `HIGH` conflict.     |
| [DAA_LVL_CRIT_ACT](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_ACT) | F3442-mode action for a `CRITICAL` conflict. |

All action parameters use `Disabled`, `Warn only`, `Return mode`, `Land mode`, `Position Hold mode`, and `Terminate`.
See [Detect and Avoid](../advanced_features/detect_and_avoid.md) for conflict-model and action-transition details.

## Remote ID Failsafe

<Badge type="tip" text="PX4 v1.18" />

The Remote ID failsafe is triggered when the [Remote ID (Open Drone ID)](../peripherals/remote_id.md) module is not detected or reports as unhealthy while the vehicle is armed.

The failsafe action and arming behaviour are both configured by the `COM_ARM_ODID` parameter:

| Parameter                                                                                                                                 | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ----------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_ARM_ODID"></a>[COM_ARM_ODID](../advanced_config/parameter_reference.md#COM_ARM_ODID) | Remote ID arming check and in-flight failsafe. `0`: Disabled (default), `1`: Warning only, `2`: Error only (prevents arming), `3`: Return, `4`: Land, `5`: Terminate.<br><br>On failsafe:<br>- `Error`, `Return`, `Land` and `Terminate` prevent arming.<br>- `Return`, `Land` and `Terminate` start the associated action/mode when airborne. |

## Parachute Health Failsafe

<Badge type="tip" text="PX4 v1.18" />

The parachute health failsafe is triggered when a [MAVLink parachute](../peripherals/parachute.md) system is missing or unhealthy while the vehicle is armed or airborne.

| Parameter                                                                                                               | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="COM_PARACHUTE"></a>[COM_PARACHUTE](../advanced_config/parameter_reference.md#COM_PARACHUTE) | Parachute system monitoring and failsafe action.<br>`0`: Disabled (default), `1`: [Warning](#act_warn), `2`: [Return](#act_return), `3`: [Land](#act_land).<br><br>- Everything but `Disabled` prevents arming with a failing check.<br>- [Return](#act_return) and [Land](#act_land) start the associated action when a failure happens in-flight. |

## Запобіжник Quad-chute

Аварійний режим для випадку, коли БЛА типу VTOL більше не може летіти у режимі фіксованого крила, наприклад, через відмову тягового мотора, датчика швидкості повітря або керованої поверхні.
If the failsafe is triggered, the vehicle will immediately switch to multicopter mode and execute the action defined in parameter [COM_QC_ACT](#COM_QC_ACT).

:::info
The quad-chute can also be triggered by sending a MAVLINK [MAV_CMD_DO_VTOL_TRANSITION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION) message with `param2` set to `1`.
:::

Параметри, які контролюють те, коли спрацює квадро-шнур, перераховані в таблиці нижче.

| Parameter                                                                                                                                                                                       | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_QC_ACT"></a>[COM_QC_ACT](../advanced_config/parameter_reference.md#COM_QC_ACT)                                                             | Дія чотирьох канатів після переходу на польот на багатокоптер. Can be set to: [Warning](#act_warn), [Return](#act_return), [Land](#act_land), [Hold](#act_hold).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| <a id="VT_FW_QC_HMAX"></a>[VT_FW_QC_HMAX](../advanced_config/parameter_reference.md#VT_FW_QC_HMAX)                               | Максимальна висота квадрокута, нижче якої не спрацьовує аварійний відпуск квадрокута. Це запобігає швидкому спуску квадрокутника на велику висоту, що може розрядити батарею (і саме це може спричинити крах). Висота є відносною до землі, дому або місцевого походження (за вподобанням, в залежності від доступності).                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| <a id="VT_QC_ALT_LOSS"></a>[VT_QC_ALT_LOSS](../advanced_config/parameter_reference.md#VT_QC_ALT_LOSS)                            | Uncommanded descent quad-chute altitude threshold.<br><br>In altitude controlled modes, such as [Hold mode](../flight_modes_fw/hold.md), [Position mode](../flight_modes_fw/position.md), [Altitude mode](../flight_modes_fw/altitude.md), or [Mission mode](../flight_modes_fw/mission.md), a vehicle should track its current "commanded" altitude setpoint. Запускається аварійний парашут у випадку, якщо транспортний засіб падає надто далеко нижче заданої точки (на величину, визначену в цьому параметрі).<br><br>Зверніть увагу, що аварійний парашут запускається лише у випадку, якщо транспортний засіб постійно втрачає висоту нижче заданої точки; він не запускається, якщо задана точка висоти зростає швидше, ніж може рухатися транспортний засіб. |
| <a id="VT_QC_T_ALT_LOSS"></a>[VT_QC_T_ALT_LOSS](../advanced_config/parameter_reference.md#VT_QC_T_ALT_LOSS) | Поріг втрати висоти для спрацьовування квадропарашута під час переходу VTOL до польоту на фіксованому крилі. Quad-chute спрацьовує, якщо транспортний засіб падає на таку висоту нижче своєї початкової висоти перед завершенням переходу.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <a id="VT_FW_MIN_ALT"></a>[VT_FW_MIN_ALT](../advanced_config/parameter_reference.md#VT_FW_MIN_ALT)                               | Мінімальна висота над домашнім місцем для польотів на фіксованих крилах. Коли висота падає нижче цієї величини під час польоту на фіксованому крилі, тригерується квадрокупол.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| <a id="VT_FW_QC_R"></a>[VT_FW_QC_R](../advanced_config/parameter_reference.md#VT_FW_QC_R)                                        | Абсолютний поріг обертання для спрацьовування квадро-шнура в режимі FW.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| <a id="VT_FW_QC_P"></a>[VT_FW_QC_P](../advanced_config/parameter_reference.md#VT_FW_QC_P)                                        | Абсолютний поріг чутливості для виклику квадро-шлюзу у режимі FW.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## High Wind Failsafe

The high wind failsafe can trigger a warning and/or other mode change when the wind speed exceeds the warning and maximum wind-speed threshold values.
The relevant parameters are listed in the table below.

| Parameter                                                                                                                                                                  | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="COM_WIND_MAX"></a>[COM_WIND_MAX](../advanced_config/parameter_reference.md#COM_WIND_MAX)                                  | Wind speed threshold that triggers failsafe action, in m/s ([COM_WIND_MAX_ACT](#COM_WIND_MAX_ACT)).                                                                                                                                                                                                                                                                                |
| <a id="COM_WIND_MAX_ACT"></a>[COM_WIND_MAX_ACT](../advanced_config/parameter_reference.md#COM_WIND_MAX_ACT) | High wind failsafe action (following [COM_WIND_MAX](#COM_WIND_MAX) trigger). Can be set to: `0`: None (Default), `1`: [Warning](#act_warn), `2`: [Hold](#act_hold), `3`: [Return](#act_return), `4`: [Terminate](#act_term), `5`: [Land](#act_land). |
| <a id="COM_WIND_WARN"></a>[COM_WIND_WARN](../advanced_config/parameter_reference.md#COM_WIND_WARN)                               | Wind speed threshold that triggers periodic failsafe warning.                                                                                                                                                                                                                                                                                                                                                                                                        |

## Виявлення відмов

The failure detector allows a vehicle to take protective actions if it unexpectedly flips, detects a motor failure, or if it is notified by an external failure detection system.

During **flight**, the failure detector can be used to trigger [flight termination](../advanced_config/flight_termination.md) if failure conditions are met, which may then launch a [parachute](../peripherals/parachute.md) or perform some other action.

:::info
Acting on a detected failure during flight is deactivated by default (enable by setting the parameter: [CBRK_FLIGHTTERM=0](#CBRK_FLIGHTTERM)).
:::

During **takeoff** the failure detector [attitude trigger](#attitude-trigger) invokes the [disarm action](#act_disarm) if the vehicle flips (disarm kills the motors but, unlike flight termination, will not launch a parachute or perform other failure actions).
Note that this check is _always enabled on takeoff_, irrespective of the `CBRK_FLIGHTTERM` parameter.

The failure detector is active in all vehicle types and modes, except for those where the vehicle is _expected_ to do flips (i.e. [Acro mode (MC)](../flight_modes_mc/acro.md), [Acro mode (FW)](../flight_modes_fw/acro.md), and [Manual (FW)](../flight_modes_fw/manual.md)).

### Altitude Loss Trigger {#altitude-loss-trigger}

<Badge type="tip" text="PX4 v1.18" /> <Badge type="tip" text="MC, VTOL only" />

The failure detector can be configured to trigger if a rotary-wing vehicle loses too much altitude below its commanded setpoint while in an altitude-controlled flight mode (such as [Position mode](../flight_modes_mc/position.md) or [Altitude mode](../flight_modes_mc/altitude.md)).

If the vehicle descends more than [FD_ALT_LOSS](#FD_ALT_LOSS) meters below the setpoint, [flight termination](../advanced_config/flight_termination.md) is triggered, which may deploy a [parachute](../peripherals/parachute.md).

| Parameter                                                                                                                                                         | Опис                                                                                                                                                                                                                       |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FD_ALT_LOSS"></a>[FD_ALT_LOSS](../advanced_config/parameter_reference.md#FD_ALT_LOSS)                            | Altitude loss threshold (m). Flight termination is triggered when the vehicle drops this far below the setpoint. Установіть значення 0 для відключення. |
| <a id="FD_ALT_LOSS_T"></a>[FD_ALT_LOSS_T](../advanced_config/parameter_reference.md#FD_ALT_LOSS_T) | Time (s) the vehicle must remain below the threshold before flight termination is triggered.                                                                                            |

### Тригер висоти

Детектор відмов може бути налаштований на спрацьовування, якщо стан автомобіля перевищує попередньо визначені значення крена та кочення протягом певного часу.

Відповідні параметри наведено нижче:

| Parameter                                                                                                                                                            | Опис                                                                                                                                                                                 |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="CBRK_FLIGHTTERM"></a>[CBRK_FLIGHTTERM](../advanced_config/parameter_reference.md#CBRK_FLIGHTTERM)                                        | Вимикач відключення польоту. Скасуйте з 121212 (типово), щоб увімкнути завершення польоту через виявлення відмови або втрату FMU. |
| <a id="FD_FAIL_P"></a>[FD_FAIL_P](../advanced_config/parameter_reference.md#FD_FAIL_P)                                     | Максимальний допустимий кут нахилу (в градусах).                                                                                                  |
| <a id="FD_FAIL_R"></a>[FD_FAIL_R](../advanced_config/parameter_reference.md#FD_FAIL_R)                                     | Максимальний допустимий кут крену (в градусах).                                                                                                   |
| <a id="FD_FAIL_P_TTRI"></a>[FD_FAIL_P_TTRI](../advanced_config/parameter_reference.md#FD_FAIL_P_TTRI) | Time to exceed [FD_FAIL_P](#FD_FAIL_P) for failure detection (default 0.3s).            |
| <a id="FD_FAIL_R_TTRI"></a>[FD_FAIL_R_TTRI](../advanced_config/parameter_reference.md#FD_FAIL_R_TTRI) | Time to exceed [FD_FAIL_R](#FD_FAIL_R) for failure detection (default 0.3s).            |

### Motor Failure Trigger

The failure detector can be configured to detect a motor failure while armed (and trigger an associated action) if the ESC current falls outside expected threshold for more than [MOTFAIL_TIME](#MOTFAIL_TIME) seconds.
Motor failures are non-latching: if the failure condition clears, the failure is cleared.

The undercurrent and overcurrent conditions are defined by:

```text
undercurrent: {esc current} < {MOTFAIL_C2T} * {motor command [0,1]} - {MOTFAIL_OFF}
overcurrent:  {esc current} > {MOTFAIL_C2T} * {motor command [0,1]} + {MOTFAIL_OFF}
```

| Parameter                                                                                                                                          | Опис                                                                                                                                                                                                                                                                                    |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FD_ACT_EN"></a>[FD_ACT_EN](../advanced_config/parameter_reference.md#FD_ACT_EN)                   | Enable/disable the motor failure trigger completely.                                                                                                                                                                                                                    |
| <a id="MOTFAIL_C2T"></a>[MOTFAIL_C2T](../advanced_config/parameter_reference.md#MOTFAIL_C2T)                                  | Slope between normalized motor command [0–1] and expected steady-state current (FD_ACT_MOT_C2T at 100%) (A/%). |
| <a id="MOTFAIL_OFF"></a>[MOTFAIL_OFF](../advanced_config/parameter_reference.md#MOTFAIL_OFF)                                  | Under/over-current detection threshold offset (A). Added to the expected current to form the upper bound. Subtracted from the expected current to form the lower bound.                                              |
|                                                                                                                                                    |                                                                                                                                                                                                                                                                                         |
| <a id="MOTFAIL_TIME"></a>[MOTFAIL_TIME](../advanced_config/parameter_reference.md#MOTFAIL_TIME)                               | Hysteresis time (s) for which the current threshold must remain exceeded before a motor failure is triggered.                                                                                                                                        |
| <a id="CA_FAILURE_MODE"></a>[CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) | Configure to not only warn about a motor failure but remove the first motor that detects a failure from the allocation effectiveness which turns off the motor and tries to operate the vehicle without it until disarming the next time.                               |

### External Automatic Trigger System (ATS)

The [failure detector](#failure-detector), if [enabled](#CBRK_FLIGHTTERM), can also be triggered by an external ATS system.
The external trigger system must be connected to flight controller port AUX5 (or MAIN5 on boards that do not have AUX ports), and is configured using the parameters below.

:::info
External ATS is required by [ASTM F3322-18](https://webstore.ansi.org/standards/astm/ASTMF332218).
One example of an ATS device is the [FruityChutes Sentinel Automatic Trigger System (SATS-MINI)](https://fruitychutes.com/uav_rpv_drone_recovery_parachutes/sentinel-automatic-trigger-system).
:::

| Parameter                                                                                                                                                               | Опис                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FD_EXT_ATS_EN"></a>[FD_EXT_ATS_EN](../advanced_config/parameter_reference.md#FD_EXT_ATS_EN)       | Enable PWM input on AUX5 or MAIN5 (depending on board) for engaging failsafe from an external automatic trigger system (ATS). Default: Disabled. |
| <a id="FD_EXT_ATS_TRIG"></a>[FD_EXT_ATS_TRIG](../advanced_config/parameter_reference.md#FD_EXT_ATS_TRIG) | The PWM threshold from external automatic trigger system for engaging failsafe. Default: 1900 ms.                                                                                      |

## Перевірки можливостей місії

A number of checks are run to ensure that a mission can only be started if it is _feasible_.
For example, the checks ensures that the first waypoint isn't too far away, and that the mission flight path doesn't conflict with any geofences.

As these are not strictly speaking "failsafes" they are documented in [Mission Mode (FW) > Mission Feasibility Checks](../flight_modes_fw/mission.md#mission-feasibility-checks) and [Mission Mode (MC) > Mission Feasibility Checks](../flight_modes_mc/mission.md#mission-feasibility-checks).

## Emergency Switches

Remote control switches can be configured (as part of _QGroundControl_ [Flight Mode Setup](../config/flight_mode.md)) to allow you to take rapid corrective action in the event of a problem or emergency; for example, to stop all motors, or activate [Return mode](#return-switch).

This section lists the available emergency switches.

### Kill Switch

A kill switch immediately stops all motor outputs — if flying, the vehicle will start to fall!

The motors will restart if the switch is reverted within 5 seconds, after which the vehicle will automatically disarm, and you will need to arm it again in order to start the motors.

:::info
There is also a [Kill Gesture](#kill-gesture), which cannot be reverted.
:::

### Arm/Disarm Switch

The arm/disarm switch is a _direct replacement_ for the default stick-based arming/disarming mechanism (and serves the same purpose: making sure there is an intentional step involved before the motors start/stop).
It might be used in preference to the default mechanism because:

- Of a preference of a switch over a stick motion.
- It avoids accidentally triggering arming/disarming in-air with a certain stick motion.
- There is no delay (it reacts immediately).

The arm/disarm switch immediately disarms (stop) motors for those [flight modes](../flight_modes/index.md#flight-modes) that _support disarming in flight_.
This includes:

- _Manual mode_
- _Acro mode_
- _Stabilized_

For modes that do not support disarming in flight, the switch is ignored during flight, but may be used after landing is detected.
This includes _Position mode_ and autonomous modes (e.g. _Mission_, _Land_ etc.).

:::info
[Auto disarm timeouts](#auto-disarming-timeouts) (e.g. via [COM_DISARM_LAND](#COM_DISARM_LAND)) are independent of the arm/disarm switch - ie even if the switch is armed the timeouts will still work.
:::

<!--
**Note** This can also be done by [manually setting](../advanced_config/parameters.md) the [RC_MAP_ARM_SW](../advanced_config/parameter_reference.md#RC_MAP_ARM_SW) parameter to the corresponding switch RC channel.
  If the switch positions are reversed, change the sign of the parameter [RC_ARMSWITCH_TH](../advanced_config/parameter_reference.md#RC_ARMSWITCH_TH) (or also change its value to alter the threshold value).
-->

### Return Switch

A return switch can be used to immediately engage [Return mode](../flight_modes/return.md).

## Kill Gesture

A kill gesture immediately stops all motor outputs — if flying, the vehicle will start to fall!

The action cannot be reverted without a reboot (this differs from a [Kill Switch](#kill-switch), where the operation can be reverted within 5 seconds).

| Parameter                                                                                                                                                               | Опис                                                                                                                                                                    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MAN_KILL_GEST_T"></a>[MAN_KILL_GEST_T](../advanced_config/parameter_reference.md#MAN_KILL_GEST_T) | Time to hold sticks in gesture position before killing the motors. Default: `-1` seconds (Disabled). |

## Arming/Disarming Settings

The [commander module](../advanced_config/parameter_reference.md#commander) has a number of parameters prefixed with `COM_ARM` that configure whether the vehicle can arm at all, and under what conditions (note that some parameters named with the prefix `COM_ARM` are used to arm other systems).
Parameters prefixed with `COM_DISARM_` affect disarming behaviour.

### Auto-Disarming Timeouts

You can set timeouts to automatically disarm a vehicle if it is too slow to takeoff, and/or after landing (disarming the vehicle removes power to the motors, so the propellers won't spin).

The [relevant parameters](../advanced_config/parameters.md) are shown below:

| Parameter                                                                                                                                             | Опис                                                                       |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | Timeout for auto-disarm after landing.                     |
| <a id="COM_DISARM_PRFLT"></a>[COM_DISARM_PRFLT](../advanced_config/parameter_reference.md#COM_DISARM_PRFLT) | Timeout for auto disarm if vehicle is too slow to takeoff. |

### Arming Pre-Conditions

These parameters can be used to set conditions that prevent arming.

| Parameter                                                                                                                                                                  | Опис                                                                                                                                                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_ARMABLE"></a>[COM_ARMABLE](../advanced_config/parameter_reference.md#COM_ARMABLE)                                                          | Enable arming (at all). `0`: Disabled, `1`: Enabled (default).                                                                                                                                                                                                  |
| <a id="COM_ARM_BAT_MIN"></a>[COM_ARM_BAT_MIN](../advanced_config/parameter_reference.md#COM_ARM_BAT_MIN)    | Minimum battery level for arming. `0`: Disabled (default). Values: `0`-`0.9`,                                                                                                                                                                                                      |
| <a id="COM_ARM_WO_GPS"></a>[COM_ARM_WO_GPS](../advanced_config/parameter_reference.md#COM_ARM_WO_GPS)       | Enable arming without GPS. `0`: Disabled, `1`: Enabled (default).                                                                                                                                                                                                                  |
| <a id="COM_ARM_MIS_REQ"></a>[COM_ARM_MIS_REQ](../advanced_config/parameter_reference.md#COM_ARM_MIS_REQ)    | Require valid mission to arm. `0`: Disabled (default), `1`: Enabled .                                                                                                                                                                                                              |
| <a id="COM_ARM_AUTH_REQ"></a>[COM_ARM_AUTH_REQ](../advanced_config/parameter_reference.md#COM_ARM_AUTH_REQ) | Requires arm authorisation from an external (MAVLink) system. Flag to allow arming (at all). `1`: Enabled, `0`: Disabled (default). Associated configuration parameters are prefixed with `COM_ARM_AUTH_`.   |
| <a id="COM_ARM_ODID"></a>[COM_ARM_ODID](../advanced_config/parameter_reference.md#COM_ARM_ODID)                                  | Remote ID arming check and in-flight failsafe. `0`: Disabled (default), `1`: Warning only, `2`: Error only, `3`: Return, `4`: Land, `5`: Terminate. See [Remote ID Failsafe](#remote-id-failsafe). |

In addition there are a number of parameters that configure system and sensor limits that make prevent arming if exceeded: [COM_CPU_MAX](../advanced_config/parameter_reference.md#COM_CPU_MAX), [COM_ARM_IMU_ACC](../advanced_config/parameter_reference.md#COM_ARM_IMU_ACC), [COM_ARM_IMU_GYR](../advanced_config/parameter_reference.md#COM_ARM_IMU_GYR), [COM_ARM_MAG_ANG](../advanced_config/parameter_reference.md#COM_ARM_MAG_ANG), [COM_ARM_MAG_STR](../advanced_config/parameter_reference.md#COM_ARM_MAG_STR).

## Подальша інформація

- [QGroundControl User Guide > Safety Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/safety.html)
