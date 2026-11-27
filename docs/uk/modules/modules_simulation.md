# Посилання на модулі: Симуляція

:::info
**This page is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4moduledoc/markdownout.py) from the source code** using the build command: `make module_documentation`.
Do not edit it directly; fix the module documentation in the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) source instead.
:::

## simulator_sih

Source: [modules/simulation/simulator_sih](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/simulator_sih)

### Опис

Цей модуль надає симулятор для квадрокоптерів та фіксованих крил, що повністю працює всередині апаратного автопілота.

Цей симулятор підписується на "actuator_outputs", які є сигналами ШІМ, наданими модулем розподілу керування.

Цей симулятор публікує сигнали датчиків, спотворені реалістичним шумом, щоб включити оцінювання стану в петлю.

### Імплементація

Симулятор реалізує рівняння руху за допомогою матричної алгебри.Використовується кватерніонне представлення для орієнтації.Для інтегрування використовується пряма схема Ейлера.Більшість змінних оголошуються глобальними в файлі .hpp, щоб уникнути переповнення стеку.

### Приклади

Pose the vehicle on the ground for accel and mag calibration (SITL only), sides named as the calibration names them.
With `rotate` it turns about the vertical with a tilt swing, as the mag calibration asks for:

```
simulator_sih orient left rotate
```

Each side in turn: 10 s still, then 10 s rotating, then level. This completes either calibration.

```
simulator_sih orient all
```

Level the vehicle and resume the flight dynamics:

```
simulator_sih orient off
```

### Usage {#simulator_sih_usage}

```
simulator_sih <command> [arguments...]
 Commands:
   start
     [-t <val>]  Terrain elevation GeoTIFF (DEM), SITL only
                 values: <file>

   orient        Pose the vehicle on the ground for calibration
     down|up|front|back|left|right|all|off Side facing down, every side in turn,
                 or level
     [rotate]    Rotate about the vertical

   stop

   status        print status info
```
