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

void msg_param_saved_get(MSG_param_hd msg_hd, int uart_read)
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

void msg_pack_response(MSG_orb_data msg_data, MSG_param_hd msg_hd, MSG_type msg_type, int uart_read)
{
    MSG_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));
    uint8_t send_message[37];
    memset(send_message, 0, sizeof(send_message));
    SETD *p;

    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WP_UPLOAD:
            setd_pack(&msg_response.setd);
            memcpy(send_message, &msg_response.setd, sizeof(msg_response.setd));
            send_message[26] = calculate_sum_check(send_message);
            send_message[26] = 0xab;
            write(uart_read, send_message, sizeof(msg_response.setd));
            break;
        case WIFI_COMM_WP_DOWNLOAD:
            p = wp_data.pop;
            for (int i = 0; i < wp_data.num; i++) {
                msg_response.setd = *p;
                memcpy(send_message, &msg_response.setd, sizeof(msg_response.setd));
                send_message[26] = calculate_sum_check(send_message);
                send_message[26] = 0xab;
                write(uart_read, send_message, sizeof(msg_response.setd));
                if(p == &wp_data.setd[19]) p = wp_data.setd;
                else p++;
            }
            break;
        case WIFI_COMM_PARAM_GET:
            msg_pack_send(msg_data, uart_read);
            break;
        case WIFI_COMM_GET_MID:
           docap_pack(&msg_response.docap, msg_hd);
           memcpy(send_message, &msg_response.docap, sizeof(msg_response.docap));
           send_message[7] = calculate_sum_check(send_message);
           send_message[7] = 0xab;
           write(uart_read, send_message, sizeof(msg_response.docap));
           break;
        default:
            break;
        }
        break;
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

void msg_orb_param_pro(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data, MSG_type msg_type, MSG_param_hd msg_hd)
{
    int paramd;
    float paramf;
    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
        case WIFI_COMM_AUTO_LAND:
        case WIFI_COMM_AUTO_TAKEOFF:
        case WIFI_COMM_GYRO_CLEAR:
        case WIFI_COMM_WP_CHAGE:
        case WIFI_COMM_MAG_CALI:
        case WIFI_COMM_HIGHT_CHAGE:
        case WIFI_COMM_RC_POS:
        case WIFI_COMM_ESC_CALI_ON:
        case WIFI_COMM_AUTO_FLIGHT_ON:
        case WIFI_COMM_AUTO_FLIGHT_OFF:
            if (msg_pd->command_pd != NULL)
            {
                    orb_publish(ORB_ID(vehicle_command), msg_pd->command_pd, &msg_data->command_data);
                    printf("Passing 2_1\n");
            }
            else
            {
                    msg_pd->command_pd = orb_advertise(ORB_ID(vehicle_command), &msg_data->command_data);
                    printf("Passing 2_2\n");
            }
            break;
        case WIFI_COMM_RECEIVER_ON:
            paramd = 0;
            param_set(msg_hd.rc_on_off_hd, &paramd);
            break;
        case WIFI_COMM_RECEIVER_OFF:
            paramd = 2;
            param_set(msg_hd.rc_on_off_hd, &paramd);
            break;
        case WIFI_COMM_GET_MID:
            paramf = msg_data->manual_data.z;
            param_set(msg_hd.hover_thrust, &paramf);
        case WIFI_COMM_DISARMED:
        case WIFI_COMM_ARMED:
            if (msg_pd->arm_pd != NULL)
            {
                    orb_publish(ORB_ID(actuator_armed), msg_pd->arm_pd, &msg_data->arm_data);
                    printf("Passing 2_1\n");
            }
            else
            {
                    msg_pd->arm_pd = orb_advertise(ORB_ID(actuator_armed), &msg_data->arm_data);
                    printf("Passing 2_2\n");
            }
            break;
        default:
            break;
        }
        break;
    case MSG_NAME_IWFI:
        if (msg_pd->manual_pd != NULL)
        {
                orb_publish(ORB_ID(manual_control_setpoint), msg_pd->manual_pd, &msg_data->manual_data);
                printf("Passing 2_1\n");
        }
        else
        {
                msg_pd->manual_pd = orb_advertise(ORB_ID(manual_control_setpoint), &msg_data->manual_data);
                printf("Passing 2_2\n");
        }
        break;
    default:
        break;
    }
}

void find_r_type( uint8_t *buffer, MSG_orb_data *msg_data,  MSG_orb_pub *msg_pd,
                        MSG_param_hd msg_hd, int uart_read)
{
    uint8_t data;
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));

    char *name = "$WIFI";
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
        if (check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
        {
            printf("Passing check\n");
            wifi_pack(buffer, msg_data, msg_type);
            msg_orb_param_pro(msg_pd, msg_data, msg_type, msg_hd);
            msg_pack_response(*msg_data, msg_hd, msg_type, uart_read);
        }
        return;
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
                if (yfwi_param_set(buffer, msg_hd)) {
                    msg_pack_response(*msg_data, msg_hd, msg_type, uart_read);
                    printf("Response Sended\n");
                }
            }
        }
        return;
    }

    name = "$IWFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing IWFI\n");
        for(int i = 5; i  < 30; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_IWFI;
        // if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer))
         if (check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
         {
             printf("Passing check\n");
             if (msg_data->manual_data.x == 0 && msg_data->manual_data.y == 0
                  && msg_data->manual_data.z == 0.5 && msg_data->manual_data.r == 0)
             {
                iwfi_pack(buffer, msg_data);
                msg_orb_param_pro(msg_pd, msg_data, msg_type, msg_hd);
             }
         }
         return;
    }

    name = "$HFMR";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing HFMR\n");
        for(int i = 5; i  < 30; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_HFMR;
        // if(buffer[29] == calculate_sum_check(buffer))
         if (buffer[29] == 0x3f)
         {
            param_reset_all();
         }
         return;
    }
}

