#ifndef _SERCOMM_CONS_H
#define _SERCOMM_CONS_H

/* how large buffers do we allocate? */
#define SERCOMM_CONS_ALLOC	256

int sercomm_puts(const char *s);
int sercomm_putchar(int c);

#endif /* _SERCOMM_CONS_H */
