# Multi-Vehicle Simulation

PX4 supports multi-vehicle simulation using the following simulators:

- [Multi-Vehicle Sim with Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with FlightGear](../sim_flightgear/multi_vehicle.md)
- [Multi-Vehicle Sim with JMAVSim](../sim_jmavsim/multi_vehicle.md)

The choice of the simulator depends on the vehicle to be simulated, how "good" the simulation needs to be (and for what features), and how many vehicles need to be simulated at a time:

- FlightGear is the most accurate simulator, and as a result the most heavy weight.
  It might be used if you need a great simulation but not too many vehicles at a time.
  It can also be used if different types of vehicles need to be simulated at the same time.
- [Gazebo](../sim_gazebo_gz/index.md) is less accurate and less heavy-weight and supports many features that aren't available for FlightGear.
  It can simulate many more vehicles at a time than FlightGear and it allows for different types of vehicles to be simulated at the same time.
  It can only be used with Ubuntu 20.04 and newer.
  Note, this is the successor of [Gazebo Classic](../sim_gazebo_classic/index.md) (below).
- [Gazebo Classic](../sim_gazebo_classic/index.md) is less accurate and less heavy-weight and supports many features and vehicles that aren't available for FlightGear.
  It can simulate many more vehicles at a time than FlightGear and it allows for different types of vehicles to be simulated at the same time.
- JMAVSim is a very light-weight simulator that supports only quadcopters.
  It is recommended if you need to support a lot of quadcopters, and the simulation only needs to be approximate.
