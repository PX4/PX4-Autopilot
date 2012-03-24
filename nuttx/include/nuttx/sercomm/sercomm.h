#ifndef _SERCOMM_H
#define _SERCOMM_H

/* SERCOMM layer on UART1 (modem UART) */

#include <osmocom/core/msgb.h>

#define SERCOMM_UART_NR	1

#define HDLC_FLAG	0x7E
#define HDLC_ESCAPE	0x7D

#define HDLC_C_UI	0x03
#define HDLC_C_P_BIT	(1 << 4)
#define HDLC_C_F_BIT	(1 << 4)

/* a low sercomm_dlci means high priority.  A high DLCI means low priority */
enum sercomm_dlci {
	SC_DLCI_HIGHEST = 0,
	SC_DLCI_DEBUG   = 4,
	SC_DLCI_L1A_L23 = 5,
	SC_DLCI_LOADER  = 9,
	SC_DLCI_CONSOLE = 10,
	SC_DLCI_ECHO    = 128,
	_SC_DLCI_MAX
};

void sercomm_init(void);
int sercomm_initialized(void);

/* User Interface: Tx */

/* user interface for transmitting messages for a given DLCI */
void sercomm_sendmsg(uint8_t dlci, struct msgb *msg);
/* how deep is the Tx queue for a given DLCI */
unsigned int sercomm_tx_queue_depth(uint8_t dlci);

/* User Interface: Rx */

/* receiving messages for a given DLCI */
typedef void (*dlci_cb_t)(uint8_t dlci, struct msgb *msg);
int sercomm_register_rx_cb(uint8_t dlci, dlci_cb_t cb);

/* Driver Interface */

/* fetch one octet of to-be-transmitted serial data. returns 0 if no more data */
int sercomm_drv_pull(uint8_t *ch);
/* the driver has received one byte, pass it into sercomm layer.
   returns 1 in case of success, 0 in case of unrecognized char */
int sercomm_drv_rx_char(uint8_t ch);

static inline struct msgb *sercomm_alloc_msgb(unsigned int len)
{
	return msgb_alloc_headroom(len+4, 4, "sercomm_tx");
}

#endif /* _SERCOMM_H */
