```
/usr/local/cuda-12/bin/nvcc benchmark.cu -I../../../../../include -use_fast_math --optimize 3 -std=c++17 && ./a.out
```