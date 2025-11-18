# RotorPy Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

RotorPy is a Python-based multirotor simulation environment with aerodynamic wrenches, useful for education and research in estimation, planning, and control for UAVs. It provides stand-alone classes and a Gym environment.

## Video

<lite-youtube videoid="L8-QZgc6Vwk" title="Demo of PX4 with RotorPy simulator"/>

<!-- TODO: add animation gif. -->

# Installation

RotorPy can be installed using `pip`:

```
pip install rotorpy[px4]
```

To install other tagged versions, see [`pyproject.toml`](https://github.com/spencerfolk/rotorpy/blob/main/pyproject.toml).

# Running the Simulation

This example will allow you to control the quadrotor through QGroundControl. If instead you want RotorPy to take control of the drone you can pass the `autopilot_controller=False` argument to the `PX4Multirotor` constructor when instantiating it. This can be useful to test perception pipelines, since the environment is controlled through RotorPy's own controllers.

First you need to build the SITL binary by setting up the toolchain and running:

	make px4_sitl

Then run the PX4 SITL binary with the command:

	PX4SIMULATOR=rotorpy PX4_SYS_AUTOSTART=10040 ./build/px4_sitl_default/bin/px4

Start the `example/px4_basic_example.py`, available [here](https://github.com/spencerfolk/rotorpy/blob/main/examples/basic_usage_px4.py) and referenced in the code below:

```python
# test_px4_sitl.py

from rotorpy.environments               import Environment
from rotorpy.trajectories.circular_traj import ThreeDCircularTraj
from rotorpy.vehicles.px4_multirotor    import PX4Multirotor
from rotorpy.vehicles.px4_sihsim_quadx_params import quad_params as sihsim_quadx
from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.trajectories.hover_traj    import HoverTraj

import numpy as np

circular_trajectory = ThreeDCircularTraj(radius=np.array([1,1,0]))
hover_trajectory = HoverTraj(x0=np.array([0, 0, 5]))

def main():
    vehicle    = PX4Multirotor(sihsim_quadx, enable_ground=True)
    controller = SE3Control(sihsim_quadx)

    env = Environment(
        vehicle    = vehicle,
        controller = controller,
        trajectory = circular_trajectory,
        sim_rate   = 100,
    )
    results = env.run(
        t_final      = 60,
        use_mocap=False,
        plot_mocap=False,
        plot_estimator=False,
        plot_imu=False,
        plot         = True,
        animate_bool = False,
        verbose      = True,
    )

    print("Done—PX4 SITL ran for", len(results["time"]), "steps")

if __name__ == '__main__':
    main()
```

<!-- TODO: Add an example of externally controlled environment -->
