#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <termios.h>

int qurt_uart_open(const char *dev, speed_t speed);
int qurt_uart_write(int fd, const char *buf, size_t len);
int qurt_uart_read(int fd, char *buf, size_t len, uint32_t timeout_us);

typedef int (*open_uart_func_t)(uint8_t, speed_t);
typedef int (*write_uart_func_t)(int, const void *, size_t);
typedef int (*read_uart_func_t)(int,  void *, size_t);

void configure_uart_callbacks(open_uart_func_t, write_uart_func_t, read_uart_func_t);

#ifdef __cplusplus
}
#endif // __cplusplus
