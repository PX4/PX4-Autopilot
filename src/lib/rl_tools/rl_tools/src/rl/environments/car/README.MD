# Simple 2D Car Environment

Build and run the `ui_server` (from the main directory of the repo as the working directory, so that it can find the `static` directory)
Linux:
```
./build/src/ui_server/ui_server 127.0.0.1 8000 car default
```
Windows:
```
.\build\bin\ui_server.exe 127.0.0.1 8000 car default
```
Navigate to the shown address e.g. [http://127.0.0.1:8000](http://127.0.0.1:8000) in your browser.

Build and run the `rl_environments_car_training_interactive` which then connects to the `ui_server`:
Linux:
```
./build/src/rl/environments/car/rl_environments_car_training_interactive
```
Windows:
```
.\build\bin\rl_environments_car_training_interactive.exe
```


# Old
GTK3 is required for the UI:
```
sudo apt-get install libgtk-3-dev
```
Optionally, if you want to try to drive the car using a gamepad, you can also install SDL2:
```
sudo apt-get install libsdl2-dev
```

In the root directory of the repo (switch the backend according to your platform, e.g. `MKL`, `ACCELERATE` or `OPEN_BLAS`):
```
mkdir build
cd build
MKL_ROOT=/opt/intel/oneapi/mkl/latest cmake .. -DCMAKE_BUILD_TYPE=Release -DRL_TOOLS_BACKEND_ENABLE_MKL=ON -DRL_TOOLS_ENABLE_GTK=ON -DRL_TOOLS_ENABLE_SDL2=ON
```

Then build the ui and training targets:
```
cmake --build . --target rl_environments_car_ui rl_environments_car_training -j$(nproc)
```

Back to the root directory of the repo:
```
cd ..
./build/src/rl/environments/car/rl_environments_car_ui
```
Wiggle your gamepad sticks to see the axes ids and restart the ui with the correct ids:
```
./build/src/rl/environments/car/rl_environments_car_ui [throttle_id] [steering_id]
```
If the axis should be inverted, add a minus in front of the id.

To run the training execute:
```
./build/src/rl/environments/car/rl_environments_car_training
```
It should learn to navigate the corners after around `100000-200000` steps which should take a couple of minutes. The training is running in the background (which is why the UI freezes for some time) and every N steps the evaluation of the current policy is run and displayed in the UI. The hyperparameters and reward function are not tuned at all. I believe it could train much faster and more reliably with better tuning.

If you would like to change the layout of the track, you can modify the `.bmp` image in `src/rl/environments/track.bmp` using e.g. Gimp (all non-black pixels are converted to drivable space).




## Docker
```
cd src/rl/environments/car/
```
```
docker build -f ../../../../examples/docker/05_wasm/Dockerfile -t rltools/wasm_build . --no-cache
```
```
docker run --rm -it --mount type=bind,source=$(cd ../../../..; pwd),target=/rl_tools,readonly --mount type=bind,source=$(cd ../../../../; pwd)/static/ui_server/car/build,target=/build --platform linux/amd64 --entrypoint /bin/bash  rltools/wasm_build
```
```
RL_TOOLS_INCLUDE_DIR=/rl_tools/include JSON_INCLUDE_DIR=/rl_tools/external/json/include RL_TOOLS_BUILD_DIR=/build /rl_tools/src/rl/environments/car/build.sh
```

## Deploy
```
scp -r $(cd ../../../../; pwd)/static/ui_server/car/* server3.jonas.es:DockerContainer/rl_tools/static/car
```


# 2025-10-22
```
em++ --version
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 4.0.17 (bb11bae9af3a6551132655a0908f86889d2d423b)
```

```
 JSON_INCLUDE_DIR=/home/jonas/mono/rl-tools/.dependencies/cmake-build-release/nlohmann_json-src/include ./build.sh
```

```
cp -r static/build/* ~/git/rl-tools.github.io/car/build/
```