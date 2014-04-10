/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>

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
