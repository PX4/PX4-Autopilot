### Debugging includes Grep all includes:
```
g++ -E src/rl/zoo/zoo.cpp -Iinclude -Iexternal/cli11/include -o -
```


### Debuggin Build Time
```
cmake .. -DRL_TOOLS_ENABLE_TARGETS=ON -DCMAKE_BUILD_TYPE=Release && cmake --build . --target clean && time cmake --build . --target rl_zoo
```
Takes about 3.5s on an i9-10885H