
#include"rw_uart.h"
#include"rw_uart_define.h"

//void send_stp (STP *stp);
uint8_t get_frame(int data){
    uint8_t frame;
    switch (data) {
//    case 5001:
//        yfpa_param->mav_type = 0;
//        break;
    case 4001:
        frame = 1;
        break;
    case 7001:
        frame = 2;
        break;
    case 6001:
        frame = 3;
        break;
    case 9001:
        frame = 4;
        break;
    case 8001:
        frame = 5;
        break;
    case 11001:
        frame = 6;
        break;
    case 14001:
        frame = 7;
        break;
    case 12001:
        frame = 8;
        break;
    default:
        frame = 0;
        break;
    }
    return frame;
}

uint8_t get_control_status(uint16_t command, uint8_t nav_state){
    if (command == 241){ //VEHICLE_CMD_PREFLIGHT_CALIBRATION
        return 7;
    }
    else if (command == 242){//VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
        return 8;
    }
    else {
        switch (nav_state) {
        case 0://NAVIGATION_STATE_MANUAL :
            return 0;
            break;
        case 4: //NAVIGATION_STATE_AUTO_LOITER :
            return 1;
            break;
        case 3: //NAVIGATION_STATE_AUTO_MISSION :
            return 2;
            break;
        case 1: //NAVIGATION_STATE_ALTCTL :
            return 3;
            break;
        case 2: //NAVIGATION_STATE_POSCTL :
            return 10;
            break;
        case 17: //NAVIGATION_STATE_TAKEOFF :
            return 5;
            break;
        case 18: //NAVIGATION_STATE_LAND :
            return 6;
            break;
        case 20: //NAVIGATION_STATE_AUTO_PRECLAND :
            return 4;
            break;
        case 5://NAVIGATION_STATE_AUTO_RTL:
            return 11;
            break;
        default:
            return 0xff;
            break;
    }
 }
}

uint8_t get_warnning (bool geofence_violated, uint8_t warning ){
    uint8_t result = 0x00;
    if (warning >0){
        result |=0x03;
    }
    if (geofence_violated == true){
        result |=0x0c;
    }
    return result;
}

void stp_pack (STP *stp, MSG_orb_data stp_data){

    stp->head[0] = '$';
    stp->head[1] = 'S';
    stp->head[2] = 'T';
    stp->head[3] = 'P';

    stp->gps_vehicle_latitude = stp_data.gps_data.lat;
    stp->gps_vehicle_longitude =stp_data.gps_data.lon;
    stp->magnet_yaw =(int16_t) (stp_data.gps_data.cog_rad / 3.14159 * 180);
    stp->gps_vx = (int16_t)(stp_data.gps_data.vel_n_m_s *100.0);
    stp->gps_vy = (int16_t)(stp_data.gps_data.vel_e_m_s *100.0);
    stp->gps_num = stp_data.gps_data.satellites_used;
    stp->total_time = (uint16_t)(stp_data.gps_data.timestamp/1000000);

    /*VEHICLE_CMD_NAV_WAYPOINT*/
    stp->gps_wp_latitude = (stp_data.command_data.command == 16) ? (float_t)stp_data.command_data.param5 : NAN;
    stp->gps_wp_longitude = (stp_data.command_data.command == 16) ? (float_t)stp_data.command_data.param6 : NAN;

    stp->wp_num = (uint8_t)stp_data.mission_data.seq_total;
    stp->mission_num = (uint8_t)stp_data.mission_data.seq_current;

    stp->control_status = get_control_status (stp_data.command_data.command, stp_data.status_data.nav_state);

    stp->rc_yaw = (uint8_t)(stp_data.manual_data.r * 50.0 + 150.0);
    stp->rc_y = (uint8_t)(stp_data.manual_data.y * 50.0 +150.0 );
    stp->rc_x = (uint8_t)(-stp_data.manual_data.x * 50.0 + 150.0);
    stp->rc_z = (uint8_t)((1-stp_data.manual_data.z) * 200.0);

    if (stp_data.status_data.nav_state < 3 /*NAVIGATION_STATE_MANUAL*/ ){
        stp->sp_yaw = stp->rc_yaw;
        stp->sp_y = stp->rc_y;
        stp->sp_x = stp->rc_x;
        stp->sp_z = stp->rc_z;
    }
    else {
         stp->sp_yaw =(uint8_t)(stp_data.local_position_sp_data.yawspeed/45.0 * 50.0 + 150.0);
         stp->sp_y = (uint8_t)(stp_data.local_position_sp_data.thrust[1] * 50.0 + 150.0);
         stp->sp_x = (uint8_t)(stp_data.local_position_sp_data.thrust[0] * 50.0 + 150.0);
         //stp->sp_z = (uint8_t)(stp_data.local_position_sp_data.thrust[2] * 50.0 + 150.0);
         stp->sp_z = (uint8_t)(-stp_data.local_position_sp_data.thrust[2] * 200);
    }
    stp->local_vz_sp = (int16_t)(stp_data.local_position_sp_data.vz * 100.0);
    stp->local_z_sp = (int16_t)(stp_data.local_position_sp_data.z * 10.0);

    float_t distance = (float_t) sqrtl(stp_data.local_position_data.z * stp_data.local_position_data.z + stp_data.local_position_data.x * stp_data.local_position_data.x
                             + stp_data.local_position_data.y * stp_data.local_position_data.y);
    stp->distance_high8 =  (uint8_t)(distance/256.0);
    stp->distance_low8 = (uint8_t)(distance);
    stp->local_vx_high8 = (int8_t)(((int16_t)(stp_data.local_position_data.vx * 100.0) & 0xff00) >> 8);
    stp->local_vx_low8 =  (int8_t)((int16_t)(stp_data.local_position_data.vx * 100.0) & 0x00ff);
    stp->local_vz_high8 = (int8_t)(((int16_t)(stp_data.local_position_data.vz * 100.0) & 0xff00) >> 8);
    stp->local_vz_low8 =  (int8_t)((int16_t)(stp_data.local_position_data.vz * 100.0) & 0x00ff);
    stp->local_vy_high8 = (int8_t)(((int16_t)(stp_data.local_position_data.vy * 100.0) & 0xff00) >> 8);
    stp->local_vy_low8 =  (int8_t)((int16_t)(stp_data.local_position_data.vy * 100.0) & 0x00ff);
    stp->receiver_status = (uint8_t)(stp_data.status_data.nav_state > 2 /*NAVIGATION_STATE_POSCTL*/ );
    stp->photo_num = 0;
    stp->acc_right = (int16_t)(-stp_data.local_position_data.ay *100.0);
    stp->acc_back = (int16_t)(-stp_data.local_position_data.ax *100.0);
    stp->acc_down = (int16_t)(-stp_data.local_position_data.az *100.0);

    stp->evv = (uint8_t)(stp_data.vibe_data.vibe[2] *500);
    //printf("vibe_data2 is %.8f\n",stp_data.vibe_data.vibe[2]);
    stp->evh = (uint8_t)(stp_data.vibe_data.vibe[1] *5000);
    //printf("vibe_data1 is %.8f\n",stp_data.vibe_data.vibe[1]);

    float q0 = stp_data.attitude_data.q[0];
    float q1 = stp_data.attitude_data.q[1];
    float q2 = stp_data.attitude_data.q[2];
    float q3 = stp_data.attitude_data.q[3];
    stp->local_roll =-(int32_t)(atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) /3.14159 *180);
    stp->local_pitch = -(int32_t)(-asinf(2*(q0*q2 - q3*q1)) /3.14159 *180);
    stp->gps_yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    stp->rc_throttle_mid = 150;
    stp->rc_pitch_mid =150;
    stp->rc_roll_mid =150;
    stp->rc_yaw_mid =150;
    stp->version =1000;
    stp->sum_check=0x00;

    stp->local_z_pressure =  (int16_t)(stp_data.air_data.baro_alt_meter * 10.0);
    stp->temprature = (int8_t)(stp_data.air_data.baro_temp_celcius);

    stp->battery_voltage = (uint16_t)(stp_data.battery_data.voltage_filtered_v/25.0 * 4096.0);
    stp->battery_usage = (uint16_t)(stp_data.battery_data.discharged_mah);
    stp->battery_current = (uint8_t)(stp_data.battery_data.current_filtered_a);

    stp->skyway_state = (stp_data.status_data.nav_state < 3) ? 0x00 : 0x06;
    if(stp_data.mission_data.finished == true) stp->skyway_state |= 0x80;
    else stp->skyway_state &= 0x7f;
    if (param_saved[18] == 3) stp->skyway_state &= 0xfe;
    else stp->skyway_state |= 0x01;

    stp->warnning = get_warnning(stp_data.geofence_data.geofence_violated, stp_data.battery_data.warning);

    //stp->flight_time = (uint16_t)(stp->total_time - stp_data.arm_data.armed_time_ms/1000);
    stp->flight_time = stp_data.arm_data.armed_time_ms/1000;
    time_t t = stp_data.gps_data.time_utc_usec *1e-6;
    struct tm *lt =localtime(&t);
    stp->year = lt->tm_year - 100;
    //printf("year is %d\n", lt->tm_year);
    stp->month = lt->tm_mon +1;
    //printf("month is %d\n", lt->tm_mon);
    stp->day = lt->tm_mday;
    //printf("day is %d\n", lt->tm_mday);
    stp->hour =lt->tm_hour;
    //printf("hour is %d\n", lt->tm_hour);
    stp->minute = lt->tm_min;
    //printf("min is %d\n", lt->tm_min);
    stp->second = lt->tm_sec;
    //printf("sec is %d\n", lt->tm_sec);

    //stp->YMDHM = (lt->tm_year - 100) *1e8 + (tm_mon +1)*1e6 + lt->tm_mday *1e4 + lt->tm_hour *1e2 + lt->tm_min;
    //stp->MM =  lt->tm_sec * 1000 + (stp_data.gps_data.time_utc_usec /1000) %100;
}

void yfpa_param_pack(YFPA_param *yfpa_param, MSG_param_hd msg_hd){
    yfpa_param->head[0]='$';
    yfpa_param->head[1]='Y';
    yfpa_param->head[2]='F';
    yfpa_param->head[3]='P';
    yfpa_param->head[4]='A';
    //yfpa_param->buflen = 54; // 不连帧头，命令编号及buflen位的数据长度，自8字节始。
    yfpa_param->buflen = 62;
    yfpa_param->command =116;
    yfpa_param->command_re = 116;
    float_t paramf;
    int paramd;
    param_get(msg_hd.roll_p_hd, &paramf);
    yfpa_param->roll_p = (uint8_t)(paramf * 510.0);
    param_get(msg_hd.roll_i_hd, &paramf);
    yfpa_param->roll_i = (uint8_t)(paramf * 5100.0);
    param_get(msg_hd.roll_d_hd, &paramf);
    yfpa_param->roll_d = (uint8_t)(paramf * 25500.0);
    param_get(msg_hd.pitch_p_hd, &paramf);
    yfpa_param->pitch_p = (uint8_t)(paramf * 510.0);
    param_get(msg_hd.pitch_i_hd, &paramf);
    yfpa_param->pitch_i = (uint8_t)(paramf * 5100.0);
    param_get(msg_hd.pitch_d_hd, &paramf);
    yfpa_param->pitch_d = (uint8_t)(paramf * 25500.0);
    param_get(msg_hd.yaw_p_hd, &paramf);
    yfpa_param->yaw_p = (uint8_t)(paramf *510.0);
    param_get(msg_hd.yaw_i_hd, &paramf);
    yfpa_param->yaw_i = (uint8_t)(paramf *1275.0);
    param_get(msg_hd.yaw_d_hd, &paramf);
    yfpa_param->yaw_d = (uint8_t)(paramf *25500.0);
    param_get(msg_hd.z_p_hd, &paramf);
    yfpa_param->z_p = (uint8_t)(paramf * 170.0);
    param_get(msg_hd.yaw_force_hd, &paramd);
    yfpa_param->yaw_mode =(uint8_t)(paramd);
    param_get(msg_hd.up_vel_max_hd, &paramf);
    yfpa_param->up_vel_max = (uint8_t)((paramf - 0.5)* 34.0);
    param_get(msg_hd.xy_vel_max_hd, &paramf);
    yfpa_param->xy_vel_max = (uint8_t)((paramf-3.0) * 15.0);
    param_get(msg_hd.roll_rate_hd, &paramf);
    yfpa_param->roll_rate = (uint8_t)(paramf);
    param_get(msg_hd.pitch_rate_hd, &paramf);
    yfpa_param->pitch_rate = (uint8_t)(paramf);
    param_get(msg_hd.yaw_rate_hd, &paramf);
    yfpa_param->yaw_rate = (uint8_t)(paramf);
    param_get(msg_hd.acc_up_max_hd, &paramf);
    yfpa_param->acc_up_max = (uint8_t)((paramf - 2.0)* 19.61);
    param_get(msg_hd.yaw_max_hd, &paramf);
    yfpa_param->yaw_max = (uint8_t)(paramf);
    param_get(msg_hd.roll_max_hd, &paramf);
    yfpa_param->roll_max = (uint8_t)(paramf * 2.83);
    param_get(msg_hd.pitch_max_hd, &paramf);
    yfpa_param->pitch_max = (uint8_t)((paramf - 20.0) * 1.59);
    param_get(msg_hd.higt_max_hd, &paramf);
    yfpa_param->higt_max = (uint16_t)( paramf * 6.5535);
    param_get(msg_hd.acc_hor_max_hd, &paramf);
    yfpa_param->acc_hor_max = (uint8_t)( (paramf - 2.0) * 19.61);
    param_get(msg_hd.dist_max_hd, &paramf);
    yfpa_param->dist_max = (uint16_t)(paramf * 6.5535);
    param_get(msg_hd.mav_type_hd, &paramd);
     yfpa_param->mav_type = get_frame(paramd);
    param_get(msg_hd.battery_n_cells_hd, &paramd);
    yfpa_param->battery_num = (uint8_t)(paramd);
    param_get(msg_hd.battery_warn_hd, &paramf);
    paramf = 3.50 + 0.55 *paramf;
    yfpa_param->battery_warn = (uint8_t)((paramf - 3.55)/0.05 *4);
    param_get(msg_hd.battery_fail_hd, &paramd);
    int fail_act = 0;
    switch (paramd) {
    case 0:
        fail_act = 0x06;
        break;
    case 1:
        fail_act = 0x04;
        break;
    case 2:
        fail_act = 0x05;
        break;
//    case 3:
//        fail_act = 0x00;
//        break;
    default:
        fail_act = 0x00;
        break;
    }
    param_get(msg_hd.rc_lost_act_hd, &paramd);
   // yfpa_param->rc_lost_act = (uint8_t)(paramd);
    if (paramd == 2) fail_act |= 0x00;
    else if (paramd == 3) fail_act |= 0x10;
    else fail_act |= 0x20;
    yfpa_param->bettery_fail = (uint8_t)(fail_act);
    yfpa_param->rc_lost_act = 1;
    param_get(msg_hd.dn_vel_max_hd, &paramf);
    yfpa_param->dn_vel_max = (uint8_t)(paramf  * 63.75);
}

void setd_pack_send (void){
    uint8_t send_message[27];
    memcpy(send_message, wp_data.push, sizeof(SETD));
    send_message[26] = calculate_sum_check(send_message, sizeof(SETD));
    write(uart_read, send_message, sizeof(SETD));
}

void exyf_response_pack(MSG_type msg_type, MSG_param_hd msg_hd){
    uint8_t send_message[13];
    memset(send_message, 0, sizeof(send_message));
    send_message[0] = '$';
    send_message[1] = 'E';
    send_message[2] = 'X';
    send_message[3] = 'Y';
    send_message[4] = 'F';
    int paramd;
    uint16_t crc;
    switch (msg_type.command) {
    case EXYF_COMM_IDLE_SPEED_GET:
        send_message[5] = 13;
        send_message[6] = 0;
        send_message[7] = 10;
        send_message[8] = 10;
        param_get(msg_hd.pwm_min_hd, &paramd);
        send_message[9] = (uint8_t)(paramd % 256);
        send_message[10] = (uint8_t)(paramd / 256);
        crc = check_crc(send_message, 13);
        send_message[11] = (uint8_t)(crc & 0x00ff);
        send_message[12] = (uint8_t)((crc & 0xff00)>>8);
        //send_message[12] = 0xba;
        write(uart_read, send_message, 13);
        break;
    case EXYF_COMM_PLANE_GET:
        send_message[5] = 12;
        send_message[6] = 0;
        send_message[7] = 19;
        send_message[8] = 19;
        param_get(msg_hd.mav_type_hd, &paramd);
        send_message[9] = get_frame(paramd);
        crc = check_crc(send_message, 12);
        send_message[10] = (uint8_t)(crc & 0x00ff);
        send_message[11] = (uint8_t)((crc & 0xff00)>>8);
        //send_message[12] = 0xbb;
        write(uart_read, send_message, 12);
        break;
    default:
        break;
    }
}

void follow_ack_pack_send(uint8_t failed){
    EXYF_FOLLOW_ACK follow_ack;
    uint8_t send_message[12];
    follow_ack.head[0] = '$';
    follow_ack.head[1] = 'E';
    follow_ack.head[2] = 'X';
    follow_ack.head[3] = 'Y';
    follow_ack.head[4] = 'F';
    follow_ack.failed = failed;
    follow_ack.command = 21;
    follow_ack.command_re =21;
    follow_ack.buflen = 12;
    memcpy(send_message, &follow_ack, sizeof(EXYF_FOLLOW_ACK));
    uint16_t crc = check_crc(send_message, 12);
    send_message[10] = (uint8_t)(crc & 0x00ff);
    send_message[11] = (uint8_t)((crc & 0xff00)>>8);
    //send_message[11] = 0xba;
    write(uart_read, send_message, sizeof(EXYF_FOLLOW_ACK));
}

void docap_pack_send (int max_min){
    DOCAP docap;
    docap.head[0] = '$';
    docap.head[1] = 'D';
    docap.head[2] = 'O';
    docap.head[3] = 'C';
    docap.head[4] = 'A';
    docap.head[5] = 'P';
    docap.max_mid = (uint8_t)max_min;
    uint8_t send_message[8];
    memcpy(send_message, &docap, sizeof(DOCAP));
    send_message[7] = calculate_sum_check(send_message, sizeof(DOCAP));
    //send_message[7] = 0xaf;
    write(uart_read, send_message, sizeof(DOCAP));
}
