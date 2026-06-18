# Project Page
## Docker
```
cd src/rl/environments/pendulum/sac/wasm
```
```
docker build -f ../../../../../../examples/docker/05_wasm/Dockerfile -t rltools/wasm_build . --no-cache
```
```
docker run --rm -it -p 8000:8000 --mount type=bind,source=$(cd ../../../../../..; pwd),target=/rl_tools,readonly --mount type=bind,source=$(cd ../../../../../..; pwd)/static/rl.tools/build,target=/build rltools/wasm_build
```
```
RL_TOOLS_INCLUDE_DIR=/rl_tools/include RL_TOOLS_BUILD_DIR=/build /rl_tools/src/rl/environments/pendulum/sac/wasm/build.sh
```

## Native
```
source ~/git/emsdk/emsdk_env.sh
```
```
mkdir build
./build.sh
```

## Host
```
git submodule update --init -- rl.tools/media
```
```
python3 -m http.server --directory rl.tools
```

## Deploy
```
scp -r rl.tools/* server3.jonas.es:DockerContainer/rl_tools/static/
```
```
cp -rL * ~/phd/projects/rl_tools/rl-tools.github.io
```
