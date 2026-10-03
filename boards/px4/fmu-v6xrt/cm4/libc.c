#include "libc.h"

void *memcpy(void *dst, const void *src, size_t n)
{
	uint8_t *d = dst;
	const uint8_t *s = src;

	while (n--) {
		*d++ = *s++;
	}

	return dst;
}

void *memset(void *dst, int c, size_t n)
{
	uint8_t *d = dst;

	while (n--) {
		*d++ = (uint8_t)c;
	}

	return dst;
}

size_t strlen(const char *s)
{
	size_t n = 0;

	while (s[n]) {
		n++;
	}

	return n;
}

int strncmp(const char *a, const char *b, size_t n)
{
	for (; n; n--, a++, b++) {
		if (*a != *b || *a == '\0') {
			return (unsigned char) * a - (unsigned char) * b;
		}
	}

	return 0;
}

size_t str_append(char *dst, size_t cap, const char *src)
{
	size_t n = strlen(dst);

	while (*src && n + 1 < cap) {
		dst[n++] = *src++;
	}

	dst[n] = '\0';
	return n;
}

size_t str_append_u32(char *dst, size_t cap, uint32_t v)
{
	char tmp[11];
	size_t i = sizeof(tmp) - 1;

	tmp[i] = '\0';

	do {
		tmp[--i] = (char)('0' + v % 10);
		v /= 10;
	} while (v && i);

	return str_append(dst, cap, &tmp[i]);
}
