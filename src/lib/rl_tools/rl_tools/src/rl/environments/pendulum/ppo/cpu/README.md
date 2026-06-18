# BLAS benchmark
CPU: AMD Ryzen 9 7945HX

```
g++ -I ../include/ ../src/rl/environments/pendulum/ppo/cpu/training.cpp -std=c++17 -Ofast -lblas -DRL_TOOLS_BACKEND_ENABLE_OPENBLAS -DBENCHMARK -o benchmark && sudo nice -n-20 hyperfine './benchmark'
```


## No BLAS
```
Time (mean ± σ):      4.060 s ±  0.075 s    [User: 4.056 s, System: 0.003 s]
Range (min … max):    3.977 s …  4.151 s    10 runs
```

## libblas3 / libblas-dev
```
Time (mean ± σ):     734.7 ms ±  13.3 ms    [User: 714.2 ms, System: 20.4 ms]
Range (min … max):   707.2 ms … 749.1 ms    10 runs


```

## libatlas3-base

```
Time (mean ± σ):      1.608 s ±  0.035 s    [User: 1.605 s, System: 0.003 s]
Range (min … max):    1.571 s …  1.671 s    10 runs
```


## libopenblas-base

```
  Time (mean ± σ):     750.7 ms ±  11.6 ms    [User: 4850.0 ms, System: 18971.9 ms]
  Range (min … max):   726.3 ms … 764.9 ms    10 runs
```


## libblis-dev
```
Time (mean ± σ):     724.6 ms ±   9.3 ms    [User: 704.1 ms, System: 20.4 ms]
Range (min … max):   715.3 ms … 748.6 ms    10 runs
```

## libmkl-dev
Options from the Intel oneAPI Math Kernel Library Link Line Advisor (sequential)
```
export MKLROOT=/opt/intel/oneapi/mkl/latest && g++ -I ../include/ -std=c++17 -Ofast ../src/rl/environments/pendulum/ppo/cpu/training.cpp -o benchmark -DRL_TOOLS_BACKEND_ENABLE_MKL -DBENCHMARK  -DMKL_ILP64  -m64  -I"${MKLROOT}/include" -m64  -Wl,--start-group ${MKLROOT}/lib/libmkl_intel_ilp64.a ${MKLROOT}/lib/libmkl_sequential.a ${MKLROOT}/lib/libmkl_core.a -Wl,--end-group -lpthread -lm -ldl
```
```
Time (mean ± σ):     566.2 ms ±   4.2 ms    [User: 564.0 ms, System: 2.1 ms]
Range (min … max):   564.3 ms … 577.9 ms    10 runs

```