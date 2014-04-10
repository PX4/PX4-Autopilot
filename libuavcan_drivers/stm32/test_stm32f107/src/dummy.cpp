/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <sys/types.h>

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

/*
 * The default pulls in 70K of garbage
 */
namespace __gnu_cxx
{

void __verbose_terminate_handler()
{
    std::abort();
}

}

__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" void __cxa_atexit(void(*)(void *), void*, void*) { }

extern "C" int __aeabi_atexit(void*, void(*)(void*), void*)
{
    return 0;
}

extern "C" int __cxa_guard_acquire(__guard* g) { return !*(char*)(g); }
extern "C" void __cxa_guard_release (__guard* g) { *(char *)g = 1; }
extern "C" void __cxa_guard_abort (__guard*) { }

/*
 * The default pulls in about 12K of garbage
 */
extern "C" void __cxa_pure_virtual()
{
    std::abort();
}


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
 * sbrk function for getting space for malloc and friends
 */
extern "C"
{

extern int _end;

caddr_t _sbrk(int incr)
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if (heap == NULL)
    {
        heap = (unsigned char *) &_end;
    }
    prev_heap = heap;
    /* check removed to show basic approach */

    heap += incr;

    return (caddr_t) prev_heap;
}

}
