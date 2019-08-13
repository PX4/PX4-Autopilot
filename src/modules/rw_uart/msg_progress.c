#include "rw_uart.h"
#include "rw_uart_define.h"

void msg_pack_send( MSG_orb_data msg_data, int uart_read)
{
    uint8_t send_message[99];
    memset(send_message, 0, sizeof(send_message));
    MSG_send msg_send;
    memset(&msg_send, 0, sizeof(msg_send));
    stp_pack(&msg_send.stp, msg_data);
    memcpy(send_message, &msg_send.stp, sizeof(msg_send.stp));
    send_message[98] = calculate_sum_check(send_message);
    send_message[98] =0xab;
   // send_message[97] = (uint8_t)(msg_data.cpu_data.ram_usage * 100.0);
    //printf("send length : %d, %d\n", sizeof(msg_send->stp), sizeof(send_message));
    write(uart_read, send_message, sizeof(msg_send.stp));
}

void msg_param_saved_get(MSG_param_hd msg_hd, int uart_read, uint8_t *param_saved)
{
    MSG_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));
    yfpa_param_pack(&msg_response.yfpa_param, msg_hd);
    memcpy(param_saved, &msg_response.yfpa_param, sizeof(msg_response.yfpa_param));
    uint16_t crc = check_crc(param_saved, 54, 8);
    param_saved[60] = (uint8_t)(crc & 0x00ff);
    param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
    param_saved[61] = 0xab;
    write(uart_read, param_saved, sizeof(msg_response.yfpa_param));
    //printf("param_saved_size:%d  param_saved[31]: %x paramsaved[32]: %x\n", sizeof(msg_response.yfpa_param), param_saved[31], param_saved[32]);
}

void msg_pack_response(MSG_param_hd msg_hd, MSG_type msg_type, int uart_read, uint8_t *param_saved)
{
    MSG_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));

    switch (msg_type.name) {
    case MSG_NAME_YFWI:
        switch (msg_type.command) {
        case YFWI_COMM_CHANGE_PARAM:
            yfpa_param_pack(&msg_response.yfpa_param, msg_hd);
            memcpy(param_saved, &msg_response.yfpa_param, sizeof(msg_response.yfpa_param));
            uint16_t crc = check_crc(param_saved, 54, 8);
            param_saved[60] = (uint8_t)(crc & 0x00ff);
            param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
            param_saved[61] = 0xab;
            write(uart_read, param_saved, sizeof(msg_response.yfpa_param));
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void find_r_type( uint8_t *buffer, MSG_orb_data *msg_data,  MSG_orb_pub *msg_pd,
                        MSG_param_hd msg_hd, int uart_read, uint8_t *param_saved)
{
    char *name = "$WIFI";
    uint8_t data;
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));

    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing WIFI\n");
        for(int i = 5; i  < 30; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_WIFI;
        msg_type.command = buffer[5];
       // if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer))
        if(check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
        {
            printf("Passing check\n");
            wifi_pack(buffer, msg_data, msg_type);
            msg_orb_pub(msg_pd, msg_data, msg_type);
        }
    }

    name = "$YFWI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing YFWI\n");
        uint8_t buflen;
        read(uart_read,&data,1);
        buflen = data;
        buffer[5] = data;
        for(int i = 6; i  < buflen + 8; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_YFWI;
        msg_type.command = buffer[6];
        //uint16_t  crc_receive = (uint16_t)buffer[buflen + 8 -1] + ((uint16_t)buffer[buflen + 8] << 8);
       // if (check_command_repeat(buffer, *msg_type) && crc_receive == check_crc(buffer, buflen, 8))
        if (check_command_repeat(buffer, msg_type) && buffer[61] == 0x3f)
        {
            printf("Passing check\n");
            if (msg_type.command == YFWI_COMM_CHANGE_PARAM) {
                if (yfwi_param_set(buffer, msg_hd, param_saved)) {
                    msg_pack_response(msg_hd, msg_type, uart_read, param_saved);
                    printf("Response Sended\n");
                }
            }
        }
    }
    //if(strncmp(buffer, "$IWFI", 5)) pack_iwfi(buffer, msg_data);
}

void msg_orb_pub(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data, MSG_type msg_type)
{
    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
        case WIFI_COMM_AUTO_LAND:
        case WIFI_COMM_AUTO_TAKEOFF:
            if (msg_pd->command_pd != NULL) {
                    orb_publish(ORB_ID(vehicle_command), msg_pd->command_pd, &msg_data->command_data);
                    printf("Passing 2_1\n");
            } else {
                    msg_pd->command_pd = orb_advertise(ORB_ID(vehicle_command), &msg_data->command_data);
                    printf("Passing 2_2\n");
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
