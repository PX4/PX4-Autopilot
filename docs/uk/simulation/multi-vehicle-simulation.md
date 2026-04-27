# Симуляція кількох рухомих засобів

PX4 підтримує симуляцію кількох рухомих засобів використовуючи наступні симулятори:

- [Multi-Vehicle Sim with Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with FlightGear](../sim_flightgear/multi_vehicle.md)
- [Multi-Vehicle Sim with JMAVSim](../sim_jmavsim/multi_vehicle.md)
- [Multi-Vehicle Sim with SIH](../sim_sih/index.md#multi-vehicle-simulation)

Вибір симулятора залежить від рухомого засобу що моделюється, наскільки "якісна" потрібна симуляція (і для яких функцій), і скільки засобів потрібно симулювати одночасно.

- FlightGear це найбільш точний симулятор і в результаті найбільш ресурсомісткий.
  Він може бути використаний, якщо вам потрібна гарна симуляція, але не надто багато засобів за раз.
  Також його можна використати, якщо потрібно симулювати різні типи рухомих засобів одночасно.
- [Gazebo](../sim_gazebo_gz/index.md) is less accurate and less heavy-weight and supports many features that aren't available for FlightGear.
  Він може симулювати набагато більше засобів за раз, ніж FlightGear, та дозволяє симулювати різні їх типи водночас.
  Він може використовуватись тільки на Ubuntu 20.04 і новіше.
  Note, this is the successor of [Gazebo Classic](../sim_gazebo_classic/index.md) (below).
- [Gazebo Classic](../sim_gazebo_classic/index.md) is less accurate and less heavy-weight and supports many features and vehicles that aren't available for FlightGear.
  Він може симулювати набагато більше засобів за раз, ніж FlightGear, та дозволяє симулювати різні їх типи водночас.
- [JMAVSim](../sim_jmavsim/index.md) is a very light-weight simulator that supports only quadcopters.
  Рекомендується у випадку, якщо вам необхідно підтримувати багато квадрокоптерів та симуляція може бути приблизна.
- [SIH](../sim_sih/index.md) is the lightest-weight option with zero external dependencies.
  Since SIH is headless and runs physics internally, it can launch many instances with minimal resource usage.
  It supports all 6 vehicle types (quad, hex, plane, tailsitter, standard VTOL, rover).
