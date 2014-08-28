/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 * Standard library stubs
 */

#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>

#if __GNUC__
# pragma GCC diagnostic ignored "-Wmissing-declarations"
#endif

void* __dso_handle;

void* operator new(size_t)
{
    std::abort();
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void* operator new[](size_t)
{
    std::abort();
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void operator delete(void*)
{
    std::abort();
}

void operator delete[](void*)
{
    std::abort();
}

/*
 * stdlibc++ workaround.
 * Default implementations will throw, which causes code size explosion.
 * These definitions override the ones defined in the stdlibc+++.
 */
namespace std
{

void __throw_bad_exception() { std::abort(); }

void __throw_bad_alloc() { std::abort(); }

void __throw_bad_cast() { std::abort(); }

void __throw_bad_typeid() { std::abort(); }

void __throw_logic_error(const char*) { std::abort(); }

void __throw_domain_error(const char*) { std::abort(); }

void __throw_invalid_argument(const char*) { std::abort(); }

void __throw_length_error(const char*) { std::abort(); }

void __throw_out_of_range(const char*) { std::abort(); }

void __throw_runtime_error(const char*) { std::abort(); }

void __throw_range_error(const char*) { std::abort(); }

void __throw_overflow_error(const char*) { std::abort(); }

void __throw_underflow_error(const char*) { std::abort(); }

void __throw_ios_failure(const char*) { std::abort(); }

void __throw_system_error(int) { std::abort(); }

void __throw_future_error(int) { std::abort(); }

void __throw_bad_function_call() { std::abort(); }

}

namespace __gnu_cxx
{

void __verbose_terminate_handler()
{
    std::abort();
}

}

/*
 * libstdc++ stubs
 */
extern "C"
{

int __aeabi_atexit(void*, void(*)(void*), void*)
{
    return 0;
}

__extension__ typedef int __guard __attribute__((mode (__DI__)));

void __cxa_atexit(void(*)(void *), void*, void*)
{
}

int __cxa_guard_acquire(__guard* g)
{
    return !*g;
}

void __cxa_guard_release (__guard* g)
{
    *g = 1;
}

void __cxa_guard_abort (__guard*)
{
}

void __cxa_pure_virtual()
{
    std::abort();
}

}

/*
 * stdio
 */
extern "C"
{

__attribute__((used))
void abort()
{
    while (true) { }
}

int _read_r(struct _reent*, int, char*, int)
{
    return -1;
}

int _lseek_r(struct _reent*, int, int, int)
{
    return -1;
}

int _write_r(struct _reent*, int, char*, int)
{
    return -1;
}

int _close_r(struct _reent*, int)
{
    return -1;
}

__attribute__((used))
caddr_t _sbrk_r(struct _reent*, int)
{
    return 0;
}

int _fstat_r(struct _reent*, int, struct stat*)
{
    return -1;
}

int _isatty_r(struct _reent*, int)
{
    return -1;
}

void _exit(int)
{
    abort();
}

pid_t _getpid(void)
{
    return 1;
}

void _kill(pid_t)
{
}

}
