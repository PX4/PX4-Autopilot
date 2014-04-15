/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <board.hpp>
#include <chip.h>

namespace
{

CCAN_MSG_OBJ_T msg_obj;

extern "C"
{

void can_rx_callback(uint8_t msg_obj_num)
{
    msg_obj.msgobj = msg_obj_num;
    LPC_CCAN_API->can_receive(&msg_obj);
    if (msg_obj_num == 1)
    {
        msg_obj.msgobj = 2;
        msg_obj.mode_id += 0x100;
        LPC_CCAN_API->can_transmit(&msg_obj);
    }
}

void can_tx_callback(uint8_t msg_obj_num)
{
    (void)msg_obj_num;
}

void can_error_callback(uint32_t error_info)
{
    (void)error_info;
}

void CAN_IRQHandler(void)
{
    LPC_CCAN_API->isr();
}

} // extern "C"

bool calculateCanBaudrate(uint32_t baud_rate, uint32_t can_api_timing_cfg[2])
{
    const uint32_t pclk = Chip_Clock_GetMainClockRate();
    const uint32_t clk_per_bit = pclk / baud_rate;
    for (uint32_t div = 0; div <= 15; div++)
    {
        for (uint32_t quanta = 1; quanta <= 32; quanta++)
        {
            for (uint32_t segs = 3; segs <= 17; segs++)
            {
                if (clk_per_bit == (segs * quanta * (div + 1)))
                {
                    segs -= 3;
                    const uint32_t seg1 = segs / 2;
                    const uint32_t seg2 = segs - seg1;
                    const uint32_t can_sjw = seg1 > 3 ? 3 : seg1;
                    can_api_timing_cfg[0] = div;
                    can_api_timing_cfg[1] = ((quanta - 1) & 0x3F) |
                                            (can_sjw & 0x03) << 6 |
                                            (seg1 & 0x0F) << 8 |
                                            (seg2 & 0x07) << 12;
                    return true;
                }
            }
        }
    }
    return false;
}

bool canInit(uint32_t baudrate)
{
    static CCAN_CALLBACKS_T callbacks =
    {
        can_rx_callback,
        can_tx_callback,
        can_error_callback,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr
    };

    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);

    static uint32_t can_api_init_table[2];
    if (!calculateCanBaudrate(baudrate, can_api_init_table))
    {
        return false;
    }

    LPC_CCAN_API->init_can(can_api_init_table, true);
    LPC_CCAN_API->config_calb(&callbacks);

    NVIC_EnableIRQ(CAN_IRQn);
    return true;
}

} // namespace

int main()
{
    if (!canInit(1000000))
    {
        while (1) { }
    }

    // Send a simple one time CAN message
    msg_obj.msgobj = 0;
    msg_obj.mode_id = 0x345;
    msg_obj.mask = 0x0;
    msg_obj.dlc = 4;
    msg_obj.data[0] = 'T'; // 0x54
    msg_obj.data[1] = 'E'; // 0x45
    msg_obj.data[2] = 'S'; // 0x53
    msg_obj.data[3] = 'T'; // 0x54
    LPC_CCAN_API->can_transmit(&msg_obj);

    // Configure message object 1 to receive all 11-bit messages 0x400-0x4FF
    msg_obj.msgobj = 1;
    msg_obj.mode_id = 0x400;
    msg_obj.mask = 0x700;
    LPC_CCAN_API->config_rxmsgobj(&msg_obj);

    while (true)
    {
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
        board::setStatusLed(true);
        for (volatile int i = 0; i < 1000000; i++) { __asm volatile ("nop"); }
        board::setStatusLed(false);
    }
}
