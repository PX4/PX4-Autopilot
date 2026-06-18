# Minimal Pendulum Example
This directory contains a minimal Pendulum swing-up example. It can be compiled without any dependencies, just pointing to the RLtools include path:

```
mkdir build
cd build
g++ -I ../include/ ../src/rl/environments/pendulum/td3/cpu/standalone.cpp -std=c++17 -Ofast -o pendulum_td3
./pendulum_td3
```

Alternatively, instead of `g++` you can also use `clang++`.

If you have e.g. OpenBLAS installed you can link to it and activate it using the `RL_TOOLS_BACKEND_ENABLE_OPENBLAS` flag:

```
mkdir build
cd build
g++ -I ../include/ ../src/rl/environments/pendulum/td3/cpu/standalone.cpp -lopenblas -DRL_TOOLS_BACKEND_ENABLE_OPENBLAS -std=c++17 -Ofast -o pendulum_td3
./pendulum_td3
```

This should provide a decent speedup, you can play with some compiler options to improve speed as well:
```
mkdir build
cd build
g++ -I ../include/ ../src/rl/environments/pendulum/td3/cpu/standalone.cpp -lopenblas -DRL_TOOLS_BACKEND_ENABLE_OPENBLAS -std=c++17 -Ofast -mavx2 -march=native -ffast-math -o pendulum_td3
sudo nice -n -20 ./pendulum_td3
```
The latter gives the process the highest priority to produce more consistent results.

In our experience by far the best results are achieved by using the Intel MKL backend (3-4x faster than using OpenBLAS):
```
mkdir build
cd build
export MKLROOT=/opt/intel/oneapi/mkl/latest/
g++ -I ../include/ ../src/rl/environments/pendulum/td3/cpu/standalone.cpp   -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_ilp64 -lmkl_gnu_thread -lmkl_core -lgomp -lpthread -lm -ldl -DMKL_ILP64  -m64  -I"${MKLROOT}/include" -DRL_TOOLS_BACKEND_ENABLE_MKL -std=c++17 -Ofast -mavx2 -o pendulum_td3
sudo LD_LIBRARY_PATH="${MKLROOT}/lib/intel64:$LD_LIBRARY_PATH" nice -n -20 ./pendulum_td3
```

To show that RLtools can operate entirely dependency free (not even depending on the C++ stdlib) we can compile it with `-nodefaultlibs` and `-nostdlib`:
```
mkdir build
cd build
g++ -I ../include/ ../src/rl/environments/pendulum/td3/cpu/bare.cpp -std=c++17 -Ofast -DRL_TOOLS_TEST_BARE -DRL_TOOLS_CONTAINERS_USE_MALLOC -nodefaultlibs -lm -lc -o pendulum_td3
ldd pendulum_td3
./pendulum_td3 && echo $?
```
The latter should show that we only depend on libc (standard C library, required for dynamic memory allocation) and libm (for math functions). Since we can not print without the C++ stdlib, we check if the training was successful through the return code `echo $?` (if it is 0 the agent has learned to swing up the pendulum).