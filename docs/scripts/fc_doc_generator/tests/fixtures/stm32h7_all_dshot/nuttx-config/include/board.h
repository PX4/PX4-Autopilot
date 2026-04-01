/* Minimal board.h fixture for fc_doc_generator tests.
 * USART1: both #define GPIO_USART1_RTS and _CTS → flow_control=True
 * USART2: only #define GPIO_USART2_RTS (no CTS) → flow_control=False
 * UART4:  GPIO_UART4_RTS appears only in a comment → flow_control=False (comment ignored)
 */

#define GPIO_USART1_RX   GPIO_USART1_RX_1   /* PA10 */
#define GPIO_USART1_TX   GPIO_USART1_TX_1   /* PA9  */
#define GPIO_USART1_RTS  GPIO_USART1_RTS_1  /* PA12 */
#define GPIO_USART1_CTS  GPIO_USART1_CTS_NSS_1  /* PA11 */

#define GPIO_USART2_RX   GPIO_USART2_RX_1   /* PA3  */
#define GPIO_USART2_TX   GPIO_USART2_TX_1   /* PA2  */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_1  /* PA1  */
/* USART2 CTS intentionally omitted — only RTS defined, so no full flow control */

/* UART4: RTS mentioned in comment only — must not count as flow control */
// GPIO_UART4_RTS   no remap   /* PC8 */
#define GPIO_UART4_RX    GPIO_UART4_RX_1    /* PA1  */
#define GPIO_UART4_TX    GPIO_UART4_TX_1    /* PA0  */
