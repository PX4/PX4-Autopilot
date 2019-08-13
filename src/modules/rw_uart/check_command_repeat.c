#include "rw_uart.h"
#include "rw_uart_define.h"

bool compare_buffer_n(const uint8_t *buffer1, const uint8_t *buffer2, int n){
    bool equal = true;
    for (int i = 0; i < n; ++i) {
        equal = equal && (*(buffer1+ i) == *(buffer2 + i));
    }
    return equal;
}

bool check_command_repeat(const uint8_t *buffer, MSG_type msg_type)
{
    bool check_ok = false;
    switch (msg_type.name) {
    case MSG_NAME_WIFI :
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
            check_ok = (buffer[5] == buffer[22] && compare_buffer_n((buffer+6), (buffer+14), 8));
            break;
        case WIFI_COMM_AUTO_LAND:
        case WIFI_COMM_AUTO_TAKEOFF:
            check_ok = (buffer[5] == buffer[6] && buffer[5] == buffer [7]);
            break;
        default:
            break;
        }
        break;
    case MSG_NAME_YFWI:
//        switch (msg_type.command){
//        case YFWI_COMM_CHANGE_PARAM:
//            check_ok = (buffer[6] == buffer [7]);
//            break;
//        default:
//            break;
//        }
        check_ok = (buffer[6] == buffer [7]);
        break;
    default:
        break;
    }
    return check_ok;
}

uint8_t calculate_sum_check (const uint8_t *send_message)
{
    uint8_t sum = 0;
    for (int i=0; i < (sizeof(send_message)-1); i++){
        sum += send_message[i];
    }
    return sum;
}

uint16_t crc16_ccitt(uint8_t data, uint16_t crc)
{
    uint16_t ccitt16 = 0x1021;
    crc ^=((uint16_t)data << 8);
    for (int i = 0; i < 8; i++)
    {
        if (crc & 0x8000){
            crc <<= 1;
            crc ^= ccitt16;
        }
        else {
            crc <<= 1;
        }
    }
    return crc;
}

uint16_t check_crc(const uint8_t *buffer, uint8_t buflen, uint8_t offset)
{
    uint16_t crc = 0;
    for (int i = 0; i < buflen + offset - 2; i++) {
     crc = crc16_ccitt(buffer[i], crc);
    }
    return  crc;
}
