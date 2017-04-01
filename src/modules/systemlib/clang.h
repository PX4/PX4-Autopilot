#ifndef __CLANG_MEMCPY
#define __CLANG_MEMCPY
void __aeabi_memcpy(void *dest, const void *src, size_t n);
void __aeabi_memcpy4(void *dest, const void *src, size_t n);
void __aeabi_memcpy8(void *dest, const void *src, size_t n);
void __aeabi_memclr(void *dest, size_t n);
void __aeabi_memclr4(void *dest, size_t n);
void __aeabi_memclr8(void *dest, size_t n);
void __aeabi_memmove(void *dest, const void *src, size_t n);
void __aeabi_memset(void *s, size_t n, int c);
#endif
