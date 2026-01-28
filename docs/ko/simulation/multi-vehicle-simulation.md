# 다중 차량 시뮬레이션

PX4는 다음 시뮬레이터를 사용하여 다중 차량 시뮬레이션을 지원합니다.

- [Multi-Vehicle Sim with Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with Gazebo Classic](../sim_gazebo_classic/multi_vehicle_simulation.md) (both with and without ROS)
- [Multi-Vehicle Sim with FlightGear](../sim_flightgear/multi_vehicle.md)
- [Multi-Vehicle Sim with JMAVSim](../sim_jmavsim/multi_vehicle.md)

시뮬레이터의 선택은 시뮬레이션할 차량, 시뮬레이션 퀄러티, 시뮬레이션 기능, 시뮬레이션 차량 대수에 따라 달라집니다.

- FlightGear는 제일 정확한 시뮬레이터이며, 따라서 제일 무거운 환경입니다.
  고 퀄러티 시뮬레이션이 필요하고, 한 번에 너무 많지 않은 차량을 시뮬레이션하는 경우에 적절합니다.
  다른 유형의 차량을 동시에 시뮬레이션 하는 경우에도 사용합니다.
- [Gazebo](../sim_gazebo_gz/index.md) is less accurate and less heavy-weight and supports many features that aren't available for FlightGear.
  It can simulate many more vehicles at a time than FlightGear and it allows for different types of vehicles to be simulated at the same time.
  It can only be used with Ubuntu 20.04 and newer.
  Note, this is the successor of [Gazebo Classic](../sim_gazebo_classic/index.md) (below).
- [Gazebo Classic](../sim_gazebo_classic/index.md) is less accurate and less heavy-weight and supports many features and vehicles that aren't available for FlightGear.
  It can simulate many more vehicles at a time than FlightGear and it allows for different types of vehicles to be simulated at the same time.
- JMAVSim은 쿼드콥터만 지원하는 초경량 시뮬레이터입니다.
  많은 쿼드콥터의 근사치를 시뮬레이션하는 경우에 권장됩니다.
