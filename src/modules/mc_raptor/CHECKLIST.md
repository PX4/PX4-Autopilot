# Checklist
1. Check the `REMAP_CRAZYFLIE` flag (this remaps the Crazyflie outputs to the PX4 SIH inputs)
2. Check the `CONTROL_MULTIPLE` setting (this controls how much faster the control loop is than the simulation/step frequency during training. This is needed to aggregate e.g. 4 steps of action history into one step of the policy input)
