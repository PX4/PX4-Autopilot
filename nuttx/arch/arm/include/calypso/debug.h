#ifndef _DEBUG_H
#define _DEBUG_H

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/*
 * Check at compile time that something is of a particular type.
 * Always evaluates to 1 so you may use it easily in comparisons.
 */
#define typecheck(type,x) \
({	type __dummy; \
	typeof(x) __dummy2; \
	(void)(&__dummy == &__dummy2); \
	1; \
})

#ifdef DEBUG
#define dputchar(x) putchar(x)
#define dputs(x) puts(x)
#define dphex(x,y) phex(x,y)
#define printd(x, args ...) printf(x, ## args)
#else
#define dputchar(x)
#define dputs(x)
#define dphex(x,y)
#define printd(x, args ...)
#endif

#endif /* _DEBUG_H */
