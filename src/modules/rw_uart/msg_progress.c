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
    //send_message[98] =0xab;
   // send_message[97] = (uint8_t)(msg_data.cpu_data.ram_usage * 100.0);
    //printf("send length : %d, %d\n", sizeof(msg_send->stp), sizeof(send_message));
    //write(uart_read, send_message, sizeof(msg_send.stp));
}

void msg_param_saved_get(MSG_param_hd msg_hd)
{
    MSG_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));
    yfpa_param_pack(&msg_response.yfpa_param, msg_hd);
    memcpy(param_saved, &msg_response.yfpa_param, sizeof(msg_response.yfpa_param));
    uint16_t crc = check_crc(param_saved, 62);
    param_saved[60] = (uint8_t)(crc & 0x00ff);
    param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
    //param_saved[61] = 0xac;
    write(uart_read, param_saved, sizeof(msg_response.yfpa_param));
}

bool set_rc_channel_max(int channel){
    struct input_rc_s value;
    int set_success = 0;
    float paramf;
    param_t channel_hd = 0;
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));

    switch (channel) {
    case 1:
        channel_hd =param_find("RC1_MAX");
        break;
    case 2:
        channel_hd =param_find("RC2_MAX");
        break;
    case 3:
        channel_hd =param_find("RC3_MAX");
        break;
    case 4:
        channel_hd =param_find("RC4_MAX");
        break;
    default:
        break;
    }
    for (int i=0; i<10; i++){
       orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
       paramf = value.values[channel - 1];
        if (paramf > 1850.0)  set_success ++;
        if (set_success == 3) {
            param_set(channel_hd, &paramf);
            docap_pack_send(channel, 2);
            orb_unsubscribe(input_rc_fd);
            return true;
        }
        usleep(1000000);
    }
    orb_unsubscribe(input_rc_fd);
    return false;
}

bool set_rc_channel_min(int channel){
    struct input_rc_s value;
    int set_success = 0;
    float paramf;
    param_t channel_hd = 0;
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));

    switch (channel) {
    case 1:
        channel_hd =param_find("RC1_MIN");
        break;
    case 2:
        channel_hd =param_find("RC2_MIN");
        break;
    case 3:
        channel_hd =param_find("RC3_MIN");
        break;
    case 4:
        channel_hd =param_find("RC4_MIN");
        break;
    default:
        break;
    }
    for (int i=0; i<10; i++){
       orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
       paramf = value.values[channel - 1];
        if (paramf < 1150.0)  set_success ++;
        if (set_success == 3) {
           param_set(channel_hd, &paramf);
           docap_pack_send(channel, 1);
           orb_unsubscribe(input_rc_fd);
           return true;
        }
        usleep(1000000);
    }
    orb_unsubscribe(input_rc_fd);
    return false;
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
    MSG_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));
    uint8_t send_message[37];
    memset(send_message, 0, sizeof(send_message));
    SETD *p;
    //bool rc_cali;
    //float paramf;

    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WP_UPLOAD:
            setd_pack(&msg_response.setd);
            memcpy(send_message, &msg_response.setd, sizeof(msg_response.setd));
            send_message[26] = calculate_sum_check(send_message, sizeof(SETD));
            //send_message[26] = 0xad;
            write(uart_read, send_message, sizeof(msg_response.setd));
            break;
        case WIFI_COMM_WP_DOWNLOAD:
            p = wp_data.setd;
            for (int i = 0; i < wp_data.num; i++) {
                msg_response.setd = *p;
                memcpy(send_message, &msg_response.setd, sizeof(msg_response.setd));
                send_message[26] = calculate_sum_check(send_message, sizeof(SETD));
                //send_message[26] = 0xae;
                write(uart_read, send_message, sizeof(msg_response.setd));
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
           docap_pack_send(0xff, 0);
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
            uint16_t crc = check_crc(param_saved, 62);
            param_saved[60] = (uint8_t)(crc & 0x00ff);
            param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
            //param_saved[61] = 0xaa;
            write(uart_read, param_saved, sizeof(msg_response.yfpa_param));
            break;
        default:
            break;
        }
        break;
    case MSG_NAME_EXYF:
        exyf_response_pack(send_message, msg_type, msg_hd);
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

void publish_arm_pd(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data)
{
    if (msg_pd->arm_disarm_pd != NULL){
        orb_publish(ORB_ID(arm_disarm), msg_pd->arm_disarm_pd, &msg_data->arm_disarm_data);
        printf("Passing 2_1\n");
    }
    else{
        msg_pd->arm_disarm_pd = orb_advertise(ORB_ID(arm_disarm), &msg_data->arm_disarm_data);
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

void msg_orb_param_pro(const uint8_t *buffer, MSG_orb_pub *msg_pd, MSG_orb_data *msg_data,
                                    MSG_param_hd msg_hd, MSG_type msg_type)
{
    int paramd;
    float paramf;

    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
            msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
            //data = ((int32_t) buffer[6]) + ((int32_t) buffer[7]<<8) +((int32_t) buffer[8]<<16) + ((int32_t)buffer[9]<<24);
            msg_data->command_data.param5 = ((double_t)*(int32_t*)((uint32_t)buffer + 6)) * 1e-7;
            //memcpy(&msg_data->command_data.param5, &buffer[6], sizeof(float_t));
            //printf("lat: %d\n", *(int32_t*)((uint32_t)buffer + 6));
            msg_data->command_data.param6 = ((double_t)*(int32_t*)((uint32_t)buffer + 10))* 1e-7;
            //printf("lon: %d\n", *(int32_t*)((uint32_t)buffer + 10));
            msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
            msg_data->command_data.param1 = 0.0;
            msg_data->command_data.param2 = 0.0;
            msg_data->command_data.param3 = 0.0;
            msg_data->command_data.param4 = 0.0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing waypoint\n");
            break;
        case WIFI_COMM_AUTO_LAND:
            msg_data->command_data.command = 21; //VEHICLE_CMD_NAV_LAND
            msg_data->command_data.param4 = 0.0;
            msg_data->command_data.param5 = ((double_t)msg_data->global_position_data.lat); //* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->global_position_data.lon); //* 1e-7;
            msg_data->command_data.param7 = 0.0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing land\n");
            break;
        case WIFI_COMM_AUTO_TAKEOFF:
            msg_data->command_data.command = 22; //VEHICLE_CMD_NAV_TAKEOFF
            msg_data->command_data.param1 = 0.0;
            msg_data->command_data.param4 = 0.0;
            msg_data->command_data.param5 = ((double_t)msg_data->global_position_data.lat); //* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->global_position_data.lon); //* 1e-7;
            msg_data->command_data.param7 =((float_t) msg_data->global_position_data.alt) +10.0; //
            printf("Global alt is %.4f\n", msg_data->global_position_data.alt);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing takeoff\n");
            break;
        case WIFI_COMM_WP_UPLOAD:
            printf("Start upload\n");
            if (wp_data.num == WP_DATA_NUM_MAX) {
                printf("Too many waypoints\n");
            }else if (*(uint16_t*)((uint32_t)buffer + 6) > WP_DATA_NUM_MAX){
                printf("Wrong waypoints sequece\n");
            }else {
                uint8_t new;
                if (wp_data.setd[*(uint16_t*)((uint32_t)buffer + 6) -1].waypoint_seq > 0){
                    new = 0;
                } else new =1;
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
                wp_data.num += new;
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
            msg_data->status_data.rc_input_mode = 0; //RC_IN_MODE_ON
            publish_status_pd(msg_pd, msg_data);
            printf("Passing reiceiver on\n");
            break;
        case WIFI_COMM_RECEIVER_OFF:
            msg_data->status_data.rc_input_mode = 1; //RC_IN_MODE_OFF
            publish_status_pd(msg_pd, msg_data);
            printf("Passing reiceiver off\n");
            break;
        case WIFI_COMM_GYRO_CLEAR:
            msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
            msg_data->command_data.param1 = 1;
            msg_data->command_data.param2 = 0;
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = 0;
            msg_data->command_data.param6 = 0;
            msg_data->command_data.param7 = 0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing gyro_calibration\n");
            break;
        case WIFI_COMM_WP_CHAGE:
            msg_data->command_data.command = 177; //CMD_DO_JUMP
            msg_data->command_data.param1 = (uint16_t)buffer[7] + ((uint16_t) buffer[8]<<8);
            msg_data->command_data.param2 = 0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing wp_chage\n");
            break;
        case WIFI_COMM_MAG_CALI:
            msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
            msg_data->command_data.param1 = 0;
            msg_data->command_data.param2 = 1;
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = 0;
            msg_data->command_data.param6 = 0;
            msg_data->command_data.param7 = 0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing mag_calibration\n");
            break;
        case WIFI_COMM_HIGHT_CHANGE:
            msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
            msg_data->command_data.param5 = ((double_t)msg_data->gps_data.lat)* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->gps_data.lon)* 1e-7;
            msg_data->command_data.param7 = ((float_t)*(int16_t*)((uint32_t)buffer + 7))/10.0;
            msg_data->command_data.param1 = 0.0;
            msg_data->command_data.param2 = 0.0;
            msg_data->command_data.param3 = 0.0;
            msg_data->command_data.param4 = 0.0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing hight_change\n");
            break;
        //case WIFI_COMM_RC_POS:
        case WIFI_COMM_ESC_CALI_ON:
            msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
            msg_data->command_data.param1 = 0;
            msg_data->command_data.param2 = 0;
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = 0;
            msg_data->command_data.param6 = 0;
            msg_data->command_data.param7 = 1;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing esc_cali\n");
            break;
        case WIFI_COMM_CALI_QUIT:
            msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
            msg_data->command_data.param1 = 0;
            msg_data->command_data.param2 = 0;
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = 0;
            msg_data->command_data.param6 = 0;
            msg_data->command_data.param7 = 0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing cali_off\n");
            break;
        case WIFI_COMM_AUTO_FLIGHT_ON:
            msg_data->command_data.command = 300; //CMD_MISSION_START
            msg_data->command_data.param1 = wp_data.setd[0].waypoint_seq;
            msg_data->command_data.param2 = wp_data.push->waypoint_seq;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing atuo_on\n");
            break;
        case WIFI_COMM_AUTO_FLIGHT_OFF:
            msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = ((double_t)msg_data->gps_data.lat)* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->gps_data.lon)* 1e-7;
            msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing auto_off\n");
            break;
        case WIFI_COMM_GET_MID:
            set_rc_channel_mid();
            printf("Passing param_set get_mid\n");
            break;
        case WIFI_COMM_DISARMED:
            msg_data->arm_disarm_data.arm_disarm = false;
            msg_data->arm_disarm_data.timestamp = hrt_absolute_time();
            publish_arm_pd(msg_pd, msg_data);
            printf("Passing disarm\n");
            break;
        case WIFI_COMM_ARMED:
            msg_data->arm_disarm_data.arm_disarm = true;
            msg_data->arm_disarm_data.timestamp = hrt_absolute_time();
            publish_arm_pd(msg_pd, msg_data);
            printf("Passing arm\n");
            break;
        case WIFI_COMM_RC_POS:
            printf("Set_rc_channel_limit Start\n");
            docap_pack_send(0, 0);
            bool set_success;
            for (int i = 1; i < 5; i++)
            {
            set_success = set_rc_channel_max(i);
            if (!set_success) {
                docap_pack_send(i, 0xff);
                printf("Set_rc_channel %d max Failed\n", i);
                break;
            }
            printf("Set_rc_channel %d max Success\n", i);
            set_success = set_rc_channel_min(i);
            if (!set_success) {
                docap_pack_send(i, 0xff);
                printf("Set_rc_channel %d min Failed\n", i);
                break;
            }
            printf("Set_rc_channel %d min Success\n", i);
            }
            docap_pack_send(0xff, 0);
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
        printf("Passing iwfi_pack\n");
        publish_manual_pd(msg_pd, msg_data);
        break;

    case MSG_NAME_EXYF:
        switch (msg_type.command) {
        case EXYF_COMM_LOITER_YAW:
            if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
            msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = (float_t)*(int32_t*)((uint32_t)buffer + 9);
            msg_data->command_data.param5 = ((double_t)msg_data->gps_data.lat)* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->gps_data.lon)* 1e-7;
            msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
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
            paramd = (int)buffer[9];
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
            msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
            msg_data->command_data.param3 = 0;
            msg_data->command_data.param4 = 0;
            msg_data->command_data.param5 = ((double_t)msg_data->gps_data.lat)* 1e-7;
            msg_data->command_data.param6 = ((double_t)msg_data->gps_data.lon)* 1e-7;
            msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0 + paramf /10.0;
            paramf = *(float_t*)((uint32_t)buffer + 11);
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
    uint8_t data;
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));

    char *name = "$WIFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing WIFI\n");
        //read(uart_read,&buffer[5], 25);
        for(int i = 5; i  < 30; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
            //usleep(100);
            printf("buffer[%d] is %x\n", i ,data);
        }
        msg_type.name =MSG_NAME_WIFI;
        msg_type.command = buffer[5];

        printf("Sum check is %d, %d\n", buffer[29], calculate_sum_check(buffer, 30));
        //if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
        if (check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
        {
            printf("Passing check\n");
            //wifi_pack(buffer, msg_data, msg_type);
            msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
            msg_pack_response(*msg_data, msg_hd, msg_type);
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
        for(int i = 6; i  < buflen; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_YFWI;
        msg_type.command = buffer[6];
        uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
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
        if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
         //if (check_command_repeat(buffer, msg_type) && buffer[29] == 0x3f)
         {
             //printf("Passing check\n");
             if (msg_data->manual_data.x == 0 && msg_data->manual_data.y == 0
                  && msg_data->manual_data.z == 0.5 && msg_data->manual_data.r == 0)
             {
                //iwfi_pack(buffer, msg_data);
                msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
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
        if(buffer[29] == calculate_sum_check(buffer, 30))
         //if (buffer[29] == 0x3f)
         {
            param_reset_all();
         }
         return;
    }

    name = "$EXYF";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXYF\n");
        uint8_t buflen;
        read(uart_read,&data,1);
        buffer[5] = data;
        read(uart_read,&data,1);
        buffer[6] = data;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        printf("Buflen is %d\n", buflen);
        for(int i = 7; i  < buflen; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
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
         return;
    }

    name = "$EXEX";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXEF\n");
        uint8_t buflen;
        read(uart_read,&data,1);
        buffer[5] = data;
        read(uart_read,&data,1);
        buffer[6] = data;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        for(int i = 7; i  < buflen; i++)
        {
            read(uart_read,&data,1);
            buffer[i] = data;
        }
        msg_type.name =MSG_NAME_EXEX;
        msg_type.command = buffer[7];
        uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
        if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
        //if (check_command_repeat(buffer, msg_type) && buffer[buflen + 8] == 0x3f)
         {
            //exex_pack(buffer, msg_data, msg_type, msg_hd);
            msg_orb_param_pro(buffer ,msg_pd, msg_data, msg_hd, msg_type);
         }
         return;
    }
}

