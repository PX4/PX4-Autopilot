# Compilation & Numerics 
- Try disabling `-ffast-math` (`-DRL_TOOLS_ENABLE_FAST_MATH:BOOL=OFF`)
- Try disabling aligned malloc (`#define RL_TOOLS_DISABLE_ALIGNED_MEMORY_ALLOCATIONS`)
- Use different compilers and different versions to assess different error messages
# Dispatch
Create dispatch logs
### Cmake
```
  target_compile_options(target PRIVATE -rdynamic -finstrument-functions)
  target_link_options(target PRIVATE -rdynamic -finstrument-functions -ldl)
```
### Scaffolding

<details>
<summary>Click to expand</summary>

```
bool enable_log = false;
extern "C" {
    __attribute__((no_instrument_function))
    void __cyg_profile_func_enter(void *this_fn, void *call_site) {
        if (enable_log){
            Dl_info info;
            if (dladdr(this_fn, &info) && info.dli_sname) {
                int status = 0;
                char* demangled = abi::__cxa_demangle(info.dli_sname, nullptr, 0, &status);
                const char* fname = (status == 0 && demangled) ? demangled : info.dli_sname;
                std::printf("Enter: %s\n", fname);
                std::free(demangled);
            } else {
                std::printf("Enter: %p\n", this_fn);
            }
        }
    }

    __attribute__((no_instrument_function))
    void __cyg_profile_func_exit(void *this_fn, void *call_site) {
    }
}
```

</details>

Then set `enable_log = true` for the interesting parts
# RL
- Check the action distribution of the actor after random initialization
# CUDA
- Make sure nothing is passed by reference into any kernel
- Pay attention to `warning #20013-D: calling a constexpr __host__ function`