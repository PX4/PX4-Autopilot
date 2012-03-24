#ifndef _UART_H
#define _UART_H

#include <stdint.h>

enum uart_baudrate {
	UART_38400,
	UART_57600,
	UART_115200,
	UART_230400,
	UART_460800,
	UART_614400,
	UART_921600,
};

void uart_init(uint8_t uart, uint8_t interrupts);
void uart_putchar_wait(uint8_t uart, int c);
int uart_putchar_nb(uint8_t uart, int c);
int uart_getchar_nb(uint8_t uart, uint8_t *ch);
int uart_tx_busy(uint8_t uart);
int uart_baudrate(uint8_t uart, enum uart_baudrate bdrt);

enum uart_irq {
	UART_IRQ_TX_EMPTY,
	UART_IRQ_RX_CHAR,
};

void uart_irq_enable(uint8_t uart, enum uart_irq irq, int on);

void uart_poll(uint8_t uart);

#endif /* _UART_H */
