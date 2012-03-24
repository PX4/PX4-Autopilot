
#ifndef _DEFINES_H
#define _DEFINES_H

#define __attribute_const__	__attribute__((__const__))

/* type properties */
#define __packed		__attribute__((packed))
#define __aligned(alignment)	__attribute__((aligned(alignment)))
#define __unused		__attribute__((unused))

/* linkage */
#define __section(name) __attribute__((section(name)))

/* force placement in zero-waitstate memory */
#define __ramtext __section(".ramtext")

#endif /* !_DEFINES_H */
