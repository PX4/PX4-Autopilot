# libuavcan_kinetis
Libuavcan platform driver for Kinetis FlexCAN

The Directory contains the Kinetis FlexCAN platform driver for Libuavcan on NuttX.

Configuation is set by the NuttX board.h for the following:

|  Driver  |      board.h     |
|--------|----------------|
| OSCERCLK | BOARD_EXTAL_FREQ |
|  busclck |  BOARD_BUS_FREQ  |

and the following commandline defines:

|      Setting      |                             Description                                           |
|-------------------|-----------------------------------------------------------------------------------|
|UAVCAN_KINETIS_NUM_IFACES   | - {1..2}  Sets the number of CAN interfaces the SW will support |
|UAVCAN_KINETIS_TIMER_NUMBER | - {0..3}  Sets the Periodic Interrupt Timer (PITn) channel |

Things that could be improved:
1. Build time command line configuration of Mailbox/FIFO and filters
2. Build time command line configuration clock source
    - Currently the driver use `const uavcan::uint8_t CLOCKSEL  = 0;` To Select OSCERCLK
3. Dynamic filter disable. There are no filter enable bits on the FlexCAN, just the number of Filters
   can be set. But this changes the memory map. So the configuration show below has been chosen.

```
/* Layout of Fifo, filters and Message buffers  */

enum { FiFo = 0 };
/* 0                       */
/* 1                       */
/* 2                       */
/* 3         Fifo          */
/* 4                       */
/* 5                       */
enum { FirstFilter = 6 };
/* 6                       */
/* 7                       */
/* 8         Filters       */
/* 9                       */
enum { NumHWFilters = 16 };
enum { NumMBinFiFoAndFilters = 10 };
/* 10                      */
/* 11                      */
/* 12                      */
/* 13 Tx Message Buffers   */
/* 14                      */
/* 15                      */
/*-- ----------------------*/
```

A dedicated example application may be added later here.
For now, please consider the following open source projects as a reference:

- https://github.com/PX4/sapog
- https://github.com/Zubax/zubax_gnss
- https://github.com/PX4/Firmware
