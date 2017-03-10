// This file adds various functions that clang doesn't link properly
// AFAIK, clang doesn't have an elegant solution for this, so this is what we gotta do...

// ----- Includes -----

// Compiler Includes
#include <string.h>
#include "clang.h"

void __aeabi_memcpy(void *dest, const void *src, size_t n)
{
	(void)memcpy(dest, src, n);
}

void __aeabi_memcpy4(void *dest, const void *src, size_t n)
{
	memcpy(dest, src, n);
}

void __aeabi_memcpy8(void *dest, const void *src, size_t n)
{
	memcpy(dest, src, n);
}

void __aeabi_memclr(void *dest, size_t n)
{
	memset(dest, 0, n);
}

void __aeabi_memclr4(void *dest, size_t n)
{
	memset(dest, 0, n);
}

void __aeabi_memclr8(void *dest, size_t n)
{
	memset(dest, 0, n);
}

void __aeabi_memmove(void *dest, const void *src, size_t n)
{
	(void)memmove(dest, src, n);
}

void __aeabi_memset(void *s, size_t n, int c)
{
	(void)memset(s, c, n);
}
