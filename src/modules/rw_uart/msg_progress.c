#include "rw_uart.h"
#include "rw_uart_define.h"

void msg_pack_send( MSG_orb_data msg_data)
{
    uint8_t send_message[99];
    memset(send_message, 0, sizeof(send_message));
    MSG_send msg_send;
    memset(&msg_send, 0, sizeof(msg_send));
    stp_pack(&msg_send.stp, msg_data);
    memcpy(send_message, &msg_send.stp, sizeof(msg_send.stp));
    send_message[98] = calculate_sum_check(send_message, sizeof(send_message));
    write(uart_read, send_message, sizeof(msg_send.stp));
}

void msg_param_saved_get(MSG_param_hd msg_hd)
{
    YFPA_param yfpa_param;
    memset(&yfpa_param, 0, sizeof(YFPA_param));
    yfpa_param_pack(&yfpa_param, msg_hd);
    memcpy(param_saved, &yfpa_param, sizeof(YFPA_param));
    uint16_t crc = check_crc(param_saved, 62);
    param_saved[60] = (uint8_t)(crc & 0x00ff);
    param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
    //param_saved[61] = 0xac;
    write(uart_read, param_saved, sizeof(YFPA_param));
}

void set_rc_channel_max_min(void){
    struct input_rc_s value;
    float max_1,max_2 ,max_3, max_4;
    max_1 = max_2 = max_3 = max_4 = 1850.0;
    float min_1,min_2 ,min_3, min_4;
    min_1 = min_2 = min_3 = min_4 =1150.0;
    param_t channel1_max = param_find("RC1_MAX");
    param_t channel2_max = param_find("RC2_MAX");
    param_t channel3_max = param_find("RC3_MAX");
    param_t channel4_max = param_find("RC4_MAX");
    param_t channel1_min = param_find("RC1_MIN");
    param_t channel2_min = param_find("RC2_MIN");
    param_t channel3_min = param_find("RC3_MIN");
    param_t channel4_min = param_find("RC4_MIN");
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));

    for (int i=0; i< 500; i++){
       orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
       max_1 = value.values[0] > max_1 ? value.values[0] : max_1;
       max_2 = value.values[1] > max_2 ? value.values[1] : max_2;
       max_3 = value.values[2] > max_3 ? value.values[2] : max_3;
       max_4 = value.values[3] > max_4 ? value.values[3] : max_4;
       min_1 = value.values[0] < min_1 ? value.values[0] : min_1;
       min_2 = value.values[1] < min_2 ? value.values[1] : min_2;
       min_3 = value.values[2] < min_3 ? value.values[2] : min_3;
       min_4 = value.values[3] < min_4 ? value.values[3] : min_4;
        usleep(10000);
    }

    param_set(channel1_max, &max_1);
    param_set(channel2_max, &max_2);
    param_set(channel3_max, &max_3);
    param_set(channel4_max, &max_4);
    param_set(channel1_min, &min_1);
    param_set(channel2_min, &min_2);
    param_set(channel3_min, &min_3);
    param_set(channel4_min, &min_4);
    orb_unsubscribe(input_rc_fd);
}

void set_rc_channel_mid(void){
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s value;
    float paramf;
    orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
    param_t rc1_mid_hd = param_find("RC1_TRIM");
    param_t rc2_mid_hd = param_find("RC2_TRIM");
    param_t rc3_mid_hd = param_find("RC3_TRIM");
    param_t rc4_mid_hd = param_find("RC4_TRIM");
    paramf = value.values[0];
    param_set(rc1_mid_hd, &paramf);
    paramf = value.values[1];
    param_set(rc2_mid_hd, &paramf);
    paramf = value.values[2];
    param_set(rc3_mid_hd, &paramf);
    paramf = value.values[3];
    param_set(rc4_mid_hd, &paramf);
    orb_unsubscribe(input_rc_fd);
}

void msg_pack_response(MSG_orb_data msg_data, MSG_param_hd msg_hd, MSG_type msg_type)
{
    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WP_UPLOAD:
            setd_pack_send();
            break;
        case WIFI_COMM_WP_DOWNLOAD:
            usleep(0);
            uint8_t send_message[27];
            memset(send_message, 0, sizeof(send_message));
            SETD *p;
            p = wp_data.setd;
            for (int i = 0; i < wp_data.num; i++) {
                memcpy(send_message, p, sizeof(SETD));
                send_message[26] = calculate_sum_check(send_message, sizeof(SETD));
                //send_message[26] = 0xae;
                write(uart_read, send_message, sizeof(SETD));
                printf("Download waypoints num %d\n", i+1);
                if (p == &wp_data.setd[WP_DATA_NUM_MAX -1]) {
                    printf("Too many waypoints\n");
                    wp_data.num =WP_DATA_NUM_MAX;
                }
                else p++;
            }
            break;
        case WIFI_COMM_PARAM_GET:
            msg_pack_send(msg_data);
            msg_param_saved_get(msg_hd);
            printf("Passing param_get\n");
            break;
        case WIFI_COMM_GET_MID:
           docap_pack_send(0);
           break;
        case WIFI_COMM_RC_POS:
            docap_pack_send(1);
            break;
        default:
            break;
        }
        break;
    case MSG_NAME_YFWI:
        switch (msg_type.command) {
        case YFWI_COMM_CHANGE_PARAM:
            usleep(0);
            YFPA_param yfpa_param;
             memset(&yfpa_param, 0, sizeof(YFPA_param));
            yfpa_param_pack(&yfpa_param, msg_hd);
            memcpy(param_saved, &yfpa_param, sizeof(YFPA_param));
            uint16_t crc = check_crc(param_saved, 62);
            param_saved[60] = (uint8_t)(crc & 0x00ff);
            param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
            //param_saved[61] = 0xaa;
            write(uart_read, param_saved, sizeof(YFPA_param));
            break;
        default:
            break;
        }
        break;
    case MSG_NAME_EXYF:
        exyf_response_pack(msg_type, msg_hd);
        break;
    default:
        break;
    }
}

void wp_save(void){
    struct mission_item_s mission_item;
    mission_item.lat = ((double)wp_data.push->lat) * 1e-7;
    mission_item.lon = ((double)wp_data.push->lon) * 1e-7;
    mission_item.altitude = ((float_t)wp_data.push->alt) / 10.0;
    mission_item.altitude_is_relative = false;
    mission_item.nav_cmd = NAV_CMD_WAYPOINT;
    mission_item.time_inside = ((float_t)wp_data.push->loiter_time);
    mission_item.acceptance_radius = (wp_data.push->turn_mode == 0) ? ((float_t)wp_data.push->photo_dis) : 0;
    //mission_item.yaw = wrap_2pi(math::radians( wp_data.push->yaw));
    printf("wait dataman\n");
    bool write_failed = dm_write(DM_KEY_WAYPOINTS_OFFBOARD_1, wp_data.push->waypoint_seq, DM_PERSIST_POWER_ON_RESET,
                                         &mission_item, sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);
    if (write_failed) printf("Waypoint saving failed\n");
    else printf("Waypoint saving success\n");
}


void publish_commander_pd(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data)
{
    if (msg_pd->command_pd != NULL){
            orb_publish(ORB_ID(vehicle_command), msg_pd->command_pd, &msg_data->command_data);
            printf("Passing 2_1\n");
    }
    else{
            msg_pd->command_pd = orb_advertise(ORB_ID(vehicle_command), &msg_data->command_data);
            printf("Passing 2_2\n");
    }
}

void publish_status_pd(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data)
{
    if (msg_pd->status_pd != NULL){
        orb_publish(ORB_ID(vehicle_status), msg_pd->status_pd, &msg_data->status_data);
        printf("Passing 2_1\n");
    }
    else{
        msg_pd->status_pd = orb_advertise(ORB_ID(vehicle_status), &msg_data->status_data);//
        printf("Passing 2_2\n");
    }
}

void publish_manual_pd(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data)
{
    if (msg_pd->manual_pd != NULL){
            orb_publish(ORB_ID(manual_control_setpoint), msg_pd->manual_pd, &msg_data->manual_data);
            printf("Passing 2_1\n");
    }
    else{
            msg_pd->manual_pd = orb_advertise(ORB_ID(manual_control_setpoint), &msg_data->manual_data);
            printf("Passing 2_2\n");
    }
}

void publish_local_position_sp_pd(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data)
{
    if (msg_pd->local_position_sp_pd != NULL){
        orb_publish(ORB_ID(vehicle_local_position_setpoint), msg_pd->local_position_sp_pd, &msg_data->local_position_sp_data);
        printf("Passing 2_1\n");
    }
    else {
        msg_pd->local_position_sp_pd = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &msg_data->local_position_sp_data);
        printf("Passing 2_2\n");
    }
}

void set_command_param(struct vehicle_command_s *command_data, int command_num, float param1, float param2,
                                        float param3, float param4, double param5, double param6, float param7)
{
    command_data->command = command_num;
    command_data->param1 = param1;
    command_data->param2 = param2;
    command_data->param3 = param3;
    command_data->param4 = param4;
    command_data->param5 = param5;
    command_data->param6 = param6;
    command_data->param7 = param7;
    command_data->target_system = 1;
    command_data->target_component =1;
    command_data->source_system =255;
    command_data->source_component =0;
    command_data->confirmation =0;
    command_data->from_external =1;
}

void msg_orb_param_pro(const uint8_t *buffer, MSG_orb_pub *msg_pd, MSG_orb_data *msg_data,
                                    MSG_param_hd msg_hd, MSG_type msg_type)
{
    int paramd;
    float paramf;

    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
            set_command_param(&msg_data->command_data, 16, 0, 0, 0, 0,
                              ((double_t)*(int32_t*)((uint32_t)buffer + 6)) * 1e-7,
                              ((double_t)*(int32_t*)((uint32_t)buffer + 10))* 1e-7,
                              ((float_t) msg_data->global_position_data.alt)/1000.0);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing waypoint\n");
            break;
        case WIFI_COMM_AUTO_LAND:
            set_command_param(&msg_data->command_data, 21, 0, 0, 0, 0,
                              ((double_t)msg_data->global_position_data.lat),
                              ((double_t)msg_data->global_position_data.lon),
                              0);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing land\n");
            break;
        case WIFI_COMM_AUTO_TAKEOFF:
            set_command_param(&msg_data->command_data, 22, 0, 0, 0, 0,
                              ((double_t)msg_data->global_position_data.lat),
                              ((double_t)msg_data->global_position_data.lon),
                              ((float_t) msg_data->global_position_data.alt) -10.0);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing takeoff\n");
            break;
        case WIFI_COMM_WP_UPLOAD:
            printf("Start upload\n");
            if (wp_data.num == WP_DATA_NUM_MAX) {
                printf("Too many waypoints\n");
            }
            if (*(uint16_t*)((uint32_t)buffer + 6) > WP_DATA_NUM_MAX){
                printf("Wrong waypoints sequece\n");
            }else {
                uint8_t newpoint;
                if (wp_data.setd[*(uint16_t*)((uint32_t)buffer + 6) -1].waypoint_seq > 0){
                    newpoint = 0;
                } else newpoint =1;
                wp_data.push = &wp_data.setd[*(uint16_t*)((uint32_t)buffer + 6) -1];
                wp_data.push->head[0] = '$';
                wp_data.push->head[1] = 'S';
                wp_data.push->head[2] = 'E';
                wp_data.push->head[3] = 'T';
                wp_data.push->head[4] = 'D';
                wp_data.push->waypoint_seq = *(uint16_t*)((uint32_t)buffer + 6);
                wp_data.push->lat = *(int32_t*)((uint32_t)buffer + 8);
                wp_data.push->lon = *(int32_t*)((uint32_t)buffer + 12);
                wp_data.push->alt = *(int32_t*)((uint32_t)buffer + 16);
                wp_data.push->loiter_time = * (uint16_t*)((uint32_t)buffer + 20);
                wp_data.push->cruise_speed = buffer[22];
                wp_data.push->photo_set = buffer[23];
                wp_data.push->photo_dis = buffer[24];
                wp_data.push->turn_mode = buffer[25];
                wp_data.num += newpoint;
                wp_save();
                if (wp_data.num == WP_DATA_NUM_MAX) printf("Waypoints num is max\n");
                //else wp_data.push = &wp_data.setd[wp_data.num];
                printf("Passing wy_upload, num is %d %d\n", wp_data.num, (wp_data.push - wp_data.setd));
            }
            break;
        case WIFI_COMM_WP_UPLOAD_NUM:
            wp_data.num = *(uint16_t*)((uint32_t)buffer + 6);
            printf("Passing wp_upload_num\n");
            break;
        case WIFI_COMM_RECEIVER_ON:
            paramd = 0;
            param_set(msg_hd.rc_on_off_hd, &paramd);
            printf("Passing reiceiver on\n");
            break;
        case WIFI_COMM_RECEIVER_OFF:
            paramd = 1; //RC_IN_MODE_OFF
            param_set(msg_hd.rc_on_off_hd, &paramd);
            printf("Passing reiceiver off\n");
            break;
        case WIFI_COMM_GYRO_CLEAR:
            set_command_param(&msg_data->command_data, 241, 1, 0, 0, 0, 0, 0, 0);//CMD_PREFLIGHT_CALIBRATION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing gyro_calibration\n");
            break;
        case WIFI_COMM_WP_CHAGE:
            set_command_param(&msg_data->command_data, 177, (uint16_t)buffer[7] + ((uint16_t) buffer[8]<<8),
                                            0, 0, 0, 0, 0, 0);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing wp_chage\n");
            break;
        case WIFI_COMM_MAG_CALI:
            set_command_param(&msg_data->command_data, 241, 0, 1, 0, 0, 0, 0, 0);//CMD_PREFLIGHT_CALIBRATION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing mag_calibration\n");
            break;
        case WIFI_COMM_HIGHT_CHANGE:
            set_command_param(&msg_data->command_data, 16, 0, 0, 0, 0,
                              ((double_t)msg_data->global_position_data.lat),
                              ((double_t)msg_data->global_position_data.lon),
                              ((float_t)*(int16_t*)((uint32_t)buffer + 7))/10.0);//VEHICLE_CMD_NAV_WAYPOINT
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing hight_change\n");
            break;
        //case WIFI_COMM_RC_POS:
        case WIFI_COMM_ESC_CALI_ON:
            set_command_param(&msg_data->command_data, 241, 0, 0, 0, 0, 0, 0, 1);//CMD_PREFLIGHT_CALIBRATION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing esc_cali\n");
            break;
        case WIFI_COMM_CALI_QUIT:
            set_command_param(&msg_data->command_data, 241, 0, 0, 0, 0, 0, 0, 0);//CMD_PREFLIGHT_CALIBRATION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing cali_off\n");
            break;
        case WIFI_COMM_AUTO_FLIGHT_ON:
            set_command_param(&msg_data->command_data, 300,  wp_data.setd[0].waypoint_seq,
                                           wp_data.setd[wp_data.num-1].waypoint_seq, 0, 0, 0, 0, 0);//VEHICLE_CMD_MISSION_START
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing atuo_on\n");
            break;
        case WIFI_COMM_AUTO_FLIGHT_OFF:
            set_command_param(&msg_data->command_data, 17, 0, 0, 0, 0,
                              ((double_t)msg_data->global_position_data.lat),
                              ((double_t)msg_data->global_position_data.lon),
                              ((float_t) msg_data->global_position_data.alt));//CMD_NAV_LOITER_UNLIM
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing auto_off\n");
            break;
        case WIFI_COMM_GET_MID:
            set_rc_channel_mid();
            printf("Passing param_set get_mid\n");
            break;
        case WIFI_COMM_DISARMED:
            set_command_param(&msg_data->command_data, 400,
                                            0, 0, 0, 0, 0, 0, 0);//VEHICLE_CMD_COMPONENT_ARM_DISARM
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing disarm\n");
            break;
        case WIFI_COMM_ARMED:
            set_command_param(&msg_data->command_data, 400,
                                            1, 0, 0, 0, 0, 0, 0);//VEHICLE_CMD_COMPONENT_ARM_DISARM
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing arm\n");
            break;
        case WIFI_COMM_RC_POS:
            printf("Set_rc_channel_limit Start\n");
            set_rc_channel_max_min();
            printf("Set_rc_channel_limit Finish\n");
            break;
        default:
            break;
        }
        break;

    case MSG_NAME_IWFI:
        msg_data->manual_data.r = ((float_t)(((uint16_t) buffer[5]<<8) + buffer [6])/65535.0 - 0.5)*2.0;
        msg_data->manual_data.y = ((float_t)(((uint16_t) buffer[7]<<8) + buffer [8])/65535.0 - 0.5)*2.0;
        msg_data->manual_data.x = ((float_t)(((uint16_t) buffer[9]<<8) + buffer [10])/65535.0 - 0.5)*2.0;
        msg_data->manual_data.z = (float_t)(((uint16_t) buffer[11]<<8) + buffer [12])/65535.0;
        msg_data->manual_data.data_source = 2; //SOURCE_MAVLINK_0
        printf("Passing iwfi_pack\n");
        publish_manual_pd(msg_pd, msg_data);
        break;

    case MSG_NAME_EXYF:
        switch (msg_type.command) {
        case EXYF_COMM_LOITER_YAW:
            if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
                set_command_param(&msg_data->command_data, 17, 0, 0, 0,
                                  (float_t)*(int32_t*)((uint32_t)buffer + 9),
                                  ((double_t)msg_data->global_position_data.lat),
                                  ((double_t)msg_data->global_position_data.lon),
                                  ((float_t) msg_data->global_position_data.alt));
            }
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing loiter_yaw\n");
            break;
        case EXYF_COMM_LOITER_MOVE:
             if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
                if (buffer[11] & 0xf0){
                    //double lat, lon;
                    float x, y;
                    if (buffer[11] & 0x30){
                        y = msg_data->local_position_sp_data.y +  (buffer[11] & 0x10 ? -1:1)*((float_t)*(uint16_t*)((uint32_t)buffer + 9)/10.0);
                        x = msg_data->local_position_sp_data.x;
                    }
                    else {
                        x = msg_data->local_position_sp_data.x+  (buffer[11] & 0x40 ? -1:1)*((float_t)*(uint16_t*)((uint32_t)buffer + 9)/10.0);
                        y = msg_data->local_position_sp_data.y;
                    }

                    //if (! map_projection_global_reproject(x, y, &lat, &lon)){
    //                    msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
    //                    msg_data->command_data.param5 = (float_t)lat;
    //                    msg_data->command_data.param6 = (float_t)lon;
    //                    msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
    //                    msg_data->command_data.param1 = 0.0;
    //                    msg_data->command_data.param2 = 0.0;
    //                    msg_data->command_data.param3 = 0.0;
    //                    msg_data->command_data.param4 = 0.0;
                    //}

                    msg_data->local_position_sp_data.x = x;
                   msg_data->local_position_sp_data.y = y;
                }
             }
             publish_local_position_sp_pd(msg_pd, msg_data);
             printf("Passing loiter_move\n");
            break;
        case EXYF_COMM_IDLE_SPEED_SET:
            paramd = (int)*(uint16_t*)((uint32_t)buffer + 9);
            param_set(msg_hd.pwm_min_hd, &paramd);
             printf("Passing idle_speed_set\n");
            break;
        case EXYF_COMM_PLANE_SET:
            paramd = find_frame(buffer[9]);
            param_set(msg_hd.mav_type_hd, &paramd);
            printf("Passing plane_set\n");
            break;
        case EXYF_COMM_FOLLOW:
            if (msg_data->status_data.nav_state != 19){  //NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
                printf("Flight mode is not follow\n");
                follow_ack_pack_send(1);
            }
            else {
                struct follow_target_s follow_target_data;
                memset(&follow_target_data, 0, sizeof(follow_target_data));
//                for(int i =0; i < 48; i++){
//                printf("Buffer[%d] is %x\n", i, buffer[i]);
//                }
                memcpy(&follow_target_data.lat, &buffer[9], sizeof(double));
                //follow_target_data.lat = follow_target_data.lat * 1e7;
                printf("lat is %.4f\n", follow_target_data.lat);
                memcpy(&follow_target_data.lon, &buffer[17], sizeof(double));
                memcpy(&follow_target_data.alt, &buffer[25], sizeof(float));
                memcpy(&follow_target_data.vy, &buffer[29], sizeof(float));
                memcpy(&follow_target_data.vx, &buffer[33], sizeof(float));
                memcpy(&follow_target_data.vz, &buffer[37], sizeof(float));
                param_t ft_location_hd = param_find("NAV_FT_FS");
                paramd = (int)buffer[41];
                param_set(ft_location_hd, &paramd);
                param_t ft_distance_hd = param_find("NAV_FT_DST");
                memcpy(&paramf , &buffer[42], sizeof(float));
                printf("Follow distiance is %.4f\n",paramf);
                param_set(ft_distance_hd, &paramf);
                follow_target_data.timestamp = hrt_absolute_time();
                if (msg_pd->follow_target_pd != NULL){
                        orb_publish(ORB_ID(follow_target), msg_pd->follow_target_pd, &follow_target_data);
                        printf("Passing 2_1\n");
                }
                else{
                        msg_pd->follow_target_pd = orb_advertise(ORB_ID(follow_target), &follow_target_data);
                        printf("Passing 2_2\n");
                }
                follow_ack_pack_send(0);
            }
            break;
        default:
            break;
        }
        break;

    case MSG_NAME_EXEX:
        switch (msg_type.command) {
        case EXEX_COMM_HIGHT_CHANGE:
            paramf = (float)*(uint16_t*)((uint32_t)buffer + 9);
            set_command_param(&msg_data->command_data, 17, 0, 0, 0, 0,
                              ((double_t)msg_data->global_position_data.lat),
                              ((double_t)msg_data->global_position_data.lon),
                              ((float_t) msg_data->global_position_data.alt)+ paramf /10.0);
            memcpy(&paramf , &buffer[11], sizeof(float));
            param_set(msg_hd.up_vel_max_hd, &paramf);
            param_set(msg_hd.dn_vel_max_hd, &paramf);
            publish_commander_pd(msg_pd, msg_data);
            break;
        default:
            printf("Passing invaild command\n");
            break;
        }
        break;

    default:
        break;
    }
}

void find_r_type( uint8_t *buffer, MSG_orb_data *msg_data,  MSG_orb_pub *msg_pd,
                        MSG_param_hd msg_hd)
{
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));

    char *name = "$WIFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing WIFI\n");
        if (read_to_buff(buffer, 5, 30)){

            msg_type.name =MSG_NAME_WIFI;
            msg_type.command = buffer[5];

            printf("Sum check is %d, %d\n", buffer[29], calculate_sum_check(buffer, 30));
            //if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
            if (check_command_repeat(buffer, msg_type) && (buffer[29] == 0x3f || buffer[29] == calculate_sum_check(buffer, 30)))
            {
                printf("Passing check\n");
                //wifi_pack(buffer, msg_data, msg_type);
                msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                msg_pack_response(*msg_data, msg_hd, msg_type);
            }
        }
        return;
    }

    name = "$YFWI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing YFWI\n");
        uint8_t buflen;
        if (!read_to_buff(buffer, 5, 6)) return;
        buflen = buffer[5];
        printf("buffer len is %d\n", buflen);
        if (read_to_buff(buffer, 6, buflen)) {
            msg_type.name =MSG_NAME_YFWI;
            msg_type.command = buffer[6];
            uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
            printf("crc receive is %x\n",crc_receive);
            if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
            //if (check_command_repeat(buffer, msg_type) && buffer[buflen + 7] == 0x3f)
            {
                printf("Passing check\n");
                if (msg_type.command == YFWI_COMM_CHANGE_PARAM) {
                    if (yfwi_param_set(buffer, msg_hd)) {
                        msg_pack_response(*msg_data, msg_hd, msg_type);
                        printf("Response Sended\n");
                    }
                }
                else {
                    yfwi_pack(buffer,msg_type, msg_hd);
                }
            }
        }
        return;
    }

    name = "$IWFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing IWFI\n");
         if (read_to_buff(buffer, 5, 30)){
            msg_type.name =MSG_NAME_IWFI;
            //if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
            if (check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
             {
                 //printf("Passing check\n");
//                 if (msg_data->manual_data.x == 0 && msg_data->manual_data.y == 0
//                      && msg_data->manual_data.z == 0.5 && msg_data->manual_data.r == 0)
                 {
                    msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                 }
             }
         }
         return;
    }

    name = "$HFMR";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing HFMR\n");
        if (read_to_buff(buffer, 5, 30)){
            msg_type.name =MSG_NAME_HFMR;
            if(buffer[29] == calculate_sum_check(buffer, 30))
             //if (buffer[29] == 0x3f)
             {
                param_reset_all();
             }
        }
         return;
    }

    name = "$EXYF";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXYF\n");
        uint8_t buflen;
        if (!read_to_buff(buffer, 5, 7)) return;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        printf("Buflen is %d\n", buflen);
        if (read_to_buff(buffer, 7, buflen)){
            msg_type.name =MSG_NAME_EXYF;
            msg_type.command = buffer[7];
            uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
            //if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
            if (check_command_repeat(buffer, msg_type) && buffer[buflen -1] == 0x3f)
             {
                //exyf_pack(buffer, msg_data, msg_type, msg_hd);
                //printf("Check passed\n");
                msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                msg_pack_response(*msg_data, msg_hd, msg_type);
             }
        }
         return;
    }

    name = "$EXEX";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXEF\n");
        uint8_t buflen;
        if (!read_to_buff(buffer, 5, 7)) return;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        if (read_to_buff(buffer, 7, buflen)){
        msg_type.name =MSG_NAME_EXEX;
        msg_type.command = buffer[7];
        uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
        if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
        //if (check_command_repeat(buffer, msg_type) && buffer[buflen + 8] == 0x3f)
         {
            msg_orb_param_pro(buffer ,msg_pd, msg_data, msg_hd, msg_type);
         }
        }
         return;
    }
}

