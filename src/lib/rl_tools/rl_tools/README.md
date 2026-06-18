<div align="center">
<img src="https://github.com/rl-tools/media/blob/master/overview.jpg"/ width=500>  
</div>

<p align="center">
  <a href="https://arxiv.org/abs/2306.03530">Paper on arXiv</a> | <a href="https://rl.tools">Live demo (browser)</a> | <a href="https://docs.rl.tools">Documentation</a> | <a href="https://zoo.rl.tools">Zoo</a> | <a href="https://studio.rl.tools">Studio</a>
  </br>
</br>
  <a href="https://github.com/rl-tools/rl-tools/actions/workflows/tests-tier3.yml">
  <img src="https://github.com/rl-tools/rl-tools/actions/workflows/tests-tier3.yml/badge.svg" alt="Unit Tests">
  </a>
  <a href="https://codecov.io/gh/rl-tools/rl-tools" >
  <img src="https://codecov.io/gh/rl-tools/rl-tools/branch/main/graph/badge.svg?token=3TJZ635O8V"/>
  </a>
  <a href="https://docs.rl.tools">
  <img src="https://img.shields.io/badge/Documentation-Read%20the%20Docs-blue.svg" alt="Documentation">
  </a>
</br>
  <a href="https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=01-Containers.ipynb">
  <img src="https://mybinder.org/badge_logo.svg" alt="Run tutorials on Binder">
  </a>
  <a href="https://colab.research.google.com/github/rl-tools/documentation/blob/master/docs/09-Python%20Interface.ipynb">
  <img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Run Example on Colab">
  </a>
</br>
<a href="https://discord.gg/kbvxCavb5h">
  <img src="https://img.shields.io/discord/1194228521216778250?label=Discord&logo=discord&logoColor=white&color=7289da" alt="Join our Discord!">
  </a>
</p>



<div align="center">
<img src="https://github.com/rl-tools/media/blob/master/acrobot-swing-up-sac.gif" alt="animated" height='200'/>
<img src="https://github.com/rl-tools/media/blob/master/racing_car.gif" alt="animated" height='200'/>
</div>
<div align="center">
    Trained on a 2020 MacBook Pro (M1) using <span style="color:#7DB9B6">RLtools</span> SAC and TD3 (respectively)
</div>
</br>

<div align="center">
  <a href="https://github.com/rl-tools/rl-tools/blob/master/src/rl/environments/mujoco/ant/ppo/cpu/training.h">
    <img src="https://github.com/rl-tools/media/blob/master/rl_tools_mujoco_ant_ppo.gif" alt="animated" height='200'/>  
  </a>
  <a href="https://github.com/rl-tools/rl-tools/blob/e033cc1d739f66d18ef685233d8dd84dddb3fe69/src/rl/zoo/ppo/bottleneck-v0.h">
    <img src="https://github.com/rl-tools/media/blob/master/bottleneck.gif" alt="animated" height='200'/>  
  </a>
</div>

<div align="center">
    Trained on a 2020 MacBook Pro (M1) using <span style="color:#7DB9B6">RLtools</span> PPO/Multi-Agent PPO
</div>
</br>

<div align="center">
  <a href="https://github.com/arplaboratory/learning-to-fly">
    <img src="https://github.com/rl-tools/media/blob/master/learning-to-fly-in-seconds.gif" alt="animated" width='350'/>  
  </a>
</div>

<div align="center">
    Trained in 18s on a 2020 MacBook Pro (M1) using <span style="color:#7DB9B6">RLtools</span> TD3
</div>
</br>


## Benchmarks

<div align="center">
<img src="https://github.com/rl-tools/media/blob/master/benchmark_horizontal_ppo.png"/ width=300>  
<img src="https://github.com/rl-tools/media/blob/master/benchmark_horizontal_sac.png"/ width=300>
</div>
<div align="center">
    Benchmarks of training the Pendulum swing-up using different RL libraries (PPO and SAC respectively)
</div>
</br>
<div align="center">
<img src="https://github.com/rl-tools/media/blob/master/benchmark_vertical.png"/ width=350>  
</div>
<div align="center">
    Benchmarks of training the Pendulum swing-up on different devices (SAC, RLtools)
</div>

</br>
<div align="center">
<img src="https://github.com/rl-tools/media/blob/master/microcontroller_inference.png"/ width=600>  
</div>
<div align="center">
    Benchmarks of the inference frequency for a two-layer [64, 64] fully-connected neural network across different microcontrollers (types and architectures).
</div>

## Quick Start
Clone this repo, then build a Zoo example:
```
g++ -std=c++17 -O3 -ffast-math -I include src/rl/zoo/l2f/sac.cpp
```
Run it `./a.out 1337` (number = seed) then run `./tools/serve.sh` to visualize the results. Open `http://localhost:8000` and navigate to the ExTrack UI to watch the quadrotor flying. 

- **macOS**: Append `-framework Accelerate -DRL_TOOLS_BACKEND_ENABLE_ACCELERATE` for fast training (~4s on M3)
- **Ubuntu**: Use `apt install libopenblas-dev` and append `-lopenblas -DRL_TOOLS_BACKEND_ENABLE_OPENBLAS` (~6s on Zen 5).

## Algorithms
| Algorithm | Example                                                                                                                                                                                                                                                                                |
|-----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **TD3**   | [Pendulum](./src/rl/environments/pendulum/td3/cpu/standalone.cpp), [Racing Car](./src/rl/environments/car/car.cpp), [MuJoCo Ant-v4](./src/rl/environments/mujoco/ant/td3/training.h), [Acrobot](./src/rl/environments/acrobot/td3/acrobot.cpp)                                         |
| **PPO**   | [Pendulum](./src/rl/environments/pendulum/ppo/cpu/training.cpp), [Racing Car](./src/rl/environments/car/training_ppo.h), [MuJoCo Ant-v4 (CPU)](./src/rl/environments/mujoco/ant/ppo/cpu/training.h), [MuJoCo Ant-v4 (CUDA)](./src/rl/environments/mujoco/ant/ppo/cuda/training_ppo.cu) |
| **Multi-Agent PPO**   | [Bottleneck](./src/rl/zoo/bottleneck-v0/ppo.h) |
| **SAC**   | [Pendulum (CPU)](./src/rl/environments/pendulum/sac/cpu/training.cpp), [Pendulum (CUDA)](./src/rl/environments/pendulum/sac/cuda/sac.cu), [Acrobot](./src/rl/environments/acrobot/sac/acrobot.cpp)                                                                                     |

## Projects Based on <span style="color:#7DB9B6">RLtools</span>
- Learning to Fly in Seconds: [GitHub](https://github.com/arplaboratory/learning-to-fly) / [arXiv](https://arxiv.org/abs/2311.13081) / [YouTube](https://youtu.be/NRD43ZA1D-4) / [IEEE Spectrum](https://spectrum.ieee.org/amp/drone-quadrotor-2667196800)
- Data-Driven System Identification of Quadrotors Subject to Motor Delays [GitHub](https://github.com/arplaboratory/data-driven-system-identification) / [arXiv](https://arxiv.org/abs/2404.07837) / [YouTube](https://youtu.be/G3WGthRx2KE) / [Project Page](https://sysid.tools)


# Getting Started
> **⚠️ Note**: Check out [Getting Started](https://docs.rl.tools/getting_started.html) in the documentation for a more thorough guide

To get started implementing your own environment please refer to [rl-tools/example](https://github.com/rl-tools/example)


# Documentation
The documentation is available at [docs.rl.tools](https://docs.rl.tools) and consists of C++ notebooks. You can also run them locally to tinker around:

```
docker run -p 8888:8888 rltools/documentation
```
After running the Docker container, open the link that is displayed in the CLI (http://127.0.0.1:8888/...) in your browser and enjoy tinkering!

| Chapter                                                                                 | Interactive Notebook                                                                                                                                                                                                                                                                               |
|-----------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Overview                              ](https://docs.rl.tools/overview.html)                 | -                                                                                                                                                                                                                                                                                                  |
| [Getting Started                       ](https://docs.rl.tools/getting_started.html)                 | -                                                                                                                                                                                                                                                                                           |
| [Containers                            ](https://docs.rl.tools/01-Containers.html)            | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=01-Containers.ipynb)                                                                                                                                                             | 
| [Multiple Dispatch                     ](https://docs.rl.tools/02-Multiple%20Dispatch.html)   | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=02-Multiple%20Dispatch.ipynb)                                                                                                                                                    | 
| [Deep Learning                         ](https://docs.rl.tools/03-Deep%20Learning.html)       | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=03-Deep%20Learning.ipynb)                                                                                                                                                        | 
| [CPU Acceleration                      ](https://docs.rl.tools/04-CPU%20Acceleration.html)    | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=04-CPU%20Acceleration.ipynb)                                                                                                                                                     | 
| [MNIST Classification                  ](https://docs.rl.tools/05-MNIST%20Classification.html) | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=05-MNIST%20Classification.ipynb)                                                                                                                                                | 
| [Deep Reinforcement Learning           ](https://docs.rl.tools/06-Deep%20Reinforcement%20Learning.html) | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=06-Deep%20Reinforcement%20Learning.ipynb)                                                                                                                                       | 
| [The Loop Interface                    ](https://docs.rl.tools/07-The%20Loop%20Interface.html) | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=07-The%20Loop%20Interface.ipynb)                                                                                                                                                | 
| [Custom Environment                    ](https://docs.rl.tools/08-Custom%20Environment.html)  | [![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/rl-tools/documentation/binder?labpath=08-Custom%20Environment.ipynb)                                                                                                                                                   | 
| [Python Interface                      ](https://docs.rl.tools/09-Python%20Interface.html)                | [![Run Example on Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/rl-tools/documentation/blob/master/docs/09-Python%20Interface.ipynb) | 


[//]: # (## Content)

[//]: # (- [Getting Started]&#40;#getting-started&#41;)

[//]: # (  - [Cloning the Repository]&#40;#cloning-the-repository&#41;)

[//]: # (  - [Docker]&#40;#docker&#41;)

[//]: # (  - [Native]&#40;#native&#41;)

[//]: # (    - [Unix &#40;Linux and macOS&#41;]&#40;#unix-linux-and-macos&#41;)

[//]: # (    - [Windows]&#40;#windows&#41;)

[//]: # (- [Embedded Platforms]&#40;#embedded-platforms&#41;)

[//]: # (- [Naming Convention]&#40;#naming-convention&#41;)

[//]: # (- [Citing]&#40;#citing&#41;)


### Python Interface

We provide Python bindings that available as `rltools` through PyPI (the pip package index). Note that using Python Gym environments can slow down the trianing significantly compared to native <span style="color:#7DB9B6">RLtools</span> environments.
```
pip install rltools gymnasium
```
Usage:
```
from rltools import SAC
import gymnasium as gym
from gymnasium.wrappers import RescaleAction

seed = 0xf00d
def env_factory():
    env = gym.make("Pendulum-v1")
    env = RescaleAction(env, -1, 1)
    env.reset(seed=seed)
    return env

sac = SAC(env_factory)
state = sac.State(seed)

finished = False
while not finished:
    finished = state.step()
```
You can find more details in the [Python Interface documentation](https://docs.rl.tools/09-Python%20Interface.html) and from the repository [rl-tools/python-interface](https://github.com/rl-tools/python-interface).

## Embedded Platforms
### Inference & Training
- [iOS](https://github.com/rl-tools/ios)
- [teensy](./embedded_platforms)
### Inference
- [Crazyflie](embedded_platforms/crazyflie)
- [ESP32](embedded_platforms)
- [PX4](embedded_platforms)

## Naming Convention
We use `snake_case` for variables/instances, functions as well as namespaces and `PascalCase` for structs/classes. Furthermore, we use upper case `SNAKE_CASE` for compile-time constants. 

## Citing
When using <span style="color:#7DB9B6">RLtools</span> in an academic work please cite our publication using the following Bibtex citation:
```
@article{eschmann_rltools_2024,
  author  = {Jonas Eschmann and Dario Albani and Giuseppe Loianno},
  title   = {RLtools: A Fast, Portable Deep Reinforcement Learning Library for Continuous Control},
  journal = {Journal of Machine Learning Research},
  year    = {2024},
  volume  = {25},
  number  = {301},
  pages   = {1--19},
  url     = {http://jmlr.org/papers/v25/24-0248.html}
}
```
