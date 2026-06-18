### Recommended: Using docker to create the release
```
docker build -t rl_tools:tests .
```
```
docker run --rm -it --mount type=bind,source=$(cd ..; pwd),target=/rl_tools,readonly rl_tools:tests
```
```
cmake .. -DCMAKE_BUILD_TYPE=Release -DRL_TOOLS_RL_ENVIRONMENTS_ENABLE_MUJOCO:BOOL=ON -DRL_TOOLS_ENABLE_TENSORBOARD:BOOL=ON -DRL_TOOLS_ENABLE_HDF5:BOOL=ON -DRL_TOOLS_ENABLE_CLI11:BOOL=ON
```


Github actions test
```
docker run -it --rm ubuntu
apt update && apt install -y git git-lfs && git lfs install && git clone --recursive https://github.com/RLtools/RLtools
apt install -y build-essential cmake libhdf5-dev libboost-dev libboost-system-dev libboost-serialization-dev libprotobuf-dev
cd RLtools
```