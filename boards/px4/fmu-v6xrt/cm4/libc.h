#pragma once

#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);
size_t strlen(const char *s);
int strncmp(const char *a, const char *b, size_t n);

/* Append src to dst (capacity cap, always NUL terminated); returns dst length. */
size_t str_append(char *dst, size_t cap, const char *src);
size_t str_append_u32(char *dst, size_t cap, uint32_t v);
