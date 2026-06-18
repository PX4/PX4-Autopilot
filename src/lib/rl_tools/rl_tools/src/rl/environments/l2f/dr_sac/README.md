```
cat experiments/2024-11-04_15-34-38/9fba3d9_dr-sac_algorithm_environment_zero-init/sac_l2f_false/0007/return.json | jq '([.parameters.dynamics.rotors[].thrust_curve | add] | add)/(.parameters.dynamics.mass*9.81)'
```