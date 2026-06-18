```
mkdir build
cd build
CC=clang-16 CXX=clang++-16 CUDA_PATH=/opt/cuda-12.1 cmake ../../../../../../../.. -DRL_TOOLS_ENABLE_TESTS:BOOL=ON -DRL_TOOLS_BACKEND_ENABLE_MKL:BOOL=ON -DRL_TOOLS_BACKEND_ENABLE_CUDA:BOOL=ON -DRL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO:BOOL=ON -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON
cmake --build . --target test_rl_environments_mujoco_ant_training_ppo_cuda -j$(nproc)
cd ..
./build/tests/src/rl/environments/mujoco/ant/cuda/test_rl_environments_mujoco_ant_training_ppo_cuda
```
