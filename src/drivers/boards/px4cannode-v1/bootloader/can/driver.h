#pragma once

#include "timer.h"

typedef enum
{
  CAN_UNKNOWN   = 0,
  CAN_125KBAUD  = 1,
  CAN_250KBAUD  = 2,
  CAN_500KBAUD  = 3,
  CAN_1MBAUD    = 4
} can_speed_t;

typedef enum
{
  CAN_Mode_Normal = 0,         // Bits 30 and 31 00
  CAN_Mode_LoopBack = 1,       // Bit 30: Loop Back Mode (Debug)
  CAN_Mode_Silent = 2,         // Bit 31: Silent Mode (Debug)
  CAN_Mode_Silent_LoopBack = 3 // Bits 30 and 31 11
} can_mode_t;

typedef enum
{
  fifoAll = 0,
  MBAll = 0,
  /*
  Receive from FIFO 1 -- filters are configured to push the messages there,
  and this is called from SysTick so needs to avoid the same FIFOs/mailboxes
  as the rest of the application.
  */

  fifoGetNodeInfo = 1,
  MBGetNodeInfo = 1,
  MBNodeStatus = 1,


} can_fifo_mailbox_t;

void can_tx(uint32_t message_id, size_t length,
            const uint8_t * message, uint8_t mailbox);
uint8_t can_rx(uint32_t * out_message_id, size_t * out_length,
               uint8_t * out_message, uint8_t fifo);
int can_init(can_speed_t speed, can_mode_t mode);
int can_autobaud(can_speed_t *can_speed, bl_timer_id tboot);

int can_speed2freq(can_speed_t);
can_speed_t can_freq2speed(int freq);
