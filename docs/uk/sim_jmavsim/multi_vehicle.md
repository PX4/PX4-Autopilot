# Симуляція кількох рухомих засобів з JMAVSim

This topic explains how to simulate multiple UAV (multicopter) vehicles using [JMAVSim](../sim_jmavsim/index.md) and SITL.
Усі екземпляри транспортних засобів починають рух з однакової позиції в симуляції.

:::tip
This is the easiest way to simulate multiple vehicles running PX4.
It is suitable for testing multi-vehicle support in _QGroundControl_ (or the [MAVSDK](https://mavsdk.mavlink.io/), etc.).
[Multi-Vehicle Simulation with Gazebo](../simulation/multi-vehicle-simulation.md) should be used for swarm simulations with many vehicles, or for testing features like computer vision that are only supported by Gazebo.
:::

## Як запустити кілька екземплярів

Для запуску кількох екземплярів (на окремих портах):

1. Збірка PX4

   ```sh
   make px4_sitl_default
   ```

2. Run **sitl_multiple_run.sh**, specifying the number of instances to start (e.g. 2):

   ```sh
   ./Tools/sitl_multiple_run.sh 2
   ```

3. Запустіть перший екземпляр в тому ж терміналі (це буде працювати на передньому плані):

   ```sh
   ./Tools/simulation/jmavsim/jmavsim_run.sh -l
   ```

4. Open a new terminal for each subsequent instance, specifying the _simulation_ TCP port for the instance:

   ```sh
   ./Tools/simulation/jmavsim/jmavsim_run.sh -p 4560 -l
   ```

   The port should be set to `4560+i` where `i` iterates for each instance (from `0` to `N-1`)

_QGroundControl_ should automatically connect to all the new vehicle instances (all GCS traffic is sent to PX4's remote UDP port: `14550` from all instances).
The vehicle that is currently being controlled is displayed in the application to bar; you can select this vehicle text to display a selection list of all of the (simulated) connected vehicle instances (`Vehicle 1`, `Vehicle 2`, etc.) and choose a new vehicle to control.

Developer APIs such as _MAVSDK_ or _MAVROS_ can connect to individual instances by listening on sequentially allocated PX4 remote UDP ports from `14540` (first instance) to `14549`.
Additional instances _all_ connect to port `14549`.

> **Tip** The **sitl_multiple_run.sh** script starts a separate process for each vehicle.
> To restart the simulations after killing one of them, you must call **sitl_multiple_run.sh** again, and also restart each of the individual instances in their own terminals.

## Додаткові ресурси

- See [Simulation](../simulation/index.md) for more information about the port configuration.
