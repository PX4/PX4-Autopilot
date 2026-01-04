# Neural Network Control

PX4 supports two ways of using neural networks for multirotor control: [`mc_nn_control`](./neural_networks/neural_networks.md) and [`mc_raptor`](./neural_networks/raptor.md).

| Use Case | [`mc_raptor`](./neural_networks/raptor.md) | [`mc_nn_control`](./neural_networks/neural_networks.md) |
|----------|-------------|-----------------|
| Pre-trained policy that adapts to any quadrotor without training | ✅ RAPTOR | ❌ |
| Train policy in PyTorch/TF | ❌ | ✅ TF Lite |
| Train policy in RLtools | ✅ | ❌ |
| Use manual control (remote) with NN policy | ❌ GPS/MoCap | ✅ Manual attitude commands |
| Load policy checkpoints from SD card | ✅ Upload via MAVLink FTP | ❌ Compiled into firmware |
| Offboard setpoints | ✅ MAVLink | ❌ |
| Internal Trajectory Generator | ✅ (Position, Lissajous) | ❌ |
