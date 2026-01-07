# Neural Network Control

PX4 supports the following mechanisms for using neural networks for multirotor control:

- [MC Neural Networks Control](../neural_networks/mc_neural_network_control.md)<Badge type="warning" text="Experimental" /> — A generic neural network module that you can modify to use different underlying neural network and training models and compile into the firmware.
- [RAPTOR: A Neural Network Module for Adaptive Quadrotor Control](../neural_networks/raptor.md)<Badge type="warning" text="Experimental" /> — An adaptive RL NN module that works well with different Quad configurations without additional training.

Generally you will select the former if you wish to experiment generally with your own neural network models and training, and the later if you have a particular interest in modules that use adaptive RL control, and therefore work will without particular tuning.
Note that both modules are experimental and provided for experimentation.
The table below provides more detail on the differences.

| Use Case                                                         | [`mc_raptor`](../neural_networks/raptor.md) | [`mc_nn_control`](../neural_networks/mc_neural_network_control.md) |
| ---------------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------ |
| Pre-trained policy that adapts to any quadrotor without training | ✓ RAPTOR                                    | ✘                                                                  |
| Train policy in PyTorch/TF                                       | ✘                                           | ✓ TF Lite                                                          |
| Train policy in RLtools                                          | ✓                                           | ✓                                                                  |
| Use manual control (remote) with NN policy                       | ✘ GPS/MoCap                                 | ✓ Manual attitude commands                                         |
| Load policy checkpoints from SD card                             | ✓ Upload via MAVLink FTP                    | ✘ Compiled into firmware                                           |
| Offboard setpoints                                               | ✓ MAVLink                                   | ✘                                                                  |
| Internal Trajectory Generator                                    | ✓ (Position, Lissajous)                     | ✘                                                                  |
