#include "rw_uart.h"
#include "rw_uart_define.h"

static double lat_save =0;
static double lon_save =0;
//static uint64_t vs_last_time = 0;

void msg_pack_send( MSG_orb_data msg_data, MSG_orb_pub *msg_pd)
{
    uint8_t send_message[99];
    memset(send_message, 0, sizeof(send_message));
    MSG_send msg_send;
    memset(&msg_send, 0, sizeof(msg_send));
    stp_pack(&msg_send.stp, msg_data);
    memcpy(send_message, &msg_send.stp, sizeof(STP));
    send_message[98] = calculate_sum_check(send_message, sizeof(STP));
    write(uart_read, send_message, sizeof(STP));
    struct dg_vehicle_status_s dg_status = {};
    memcpy(&dg_status.stp, send_message, sizeof(STP));
    dg_status.timestamp = hrt_absolute_time();
    if (msg_pd->dg_vehicle_status_pd != NULL){
            orb_publish(ORB_ID(dg_vehicle_status), msg_pd->dg_vehicle_status_pd, &dg_status);
            //printf("Passing 2_1\n");
    }
    else{
            msg_pd->dg_vehicle_status_pd = orb_advertise(ORB_ID(dg_vehicle_status), &dg_status);
            //printf("Passing 2_2\n");
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

void set_rc_channel_max_min(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data){
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s value;
    float max_1,max_2 ,max_3, max_4;
    max_1 = max_2 = max_3 = max_4 = 1850.0;;
    float min_1,min_2 ,min_3, min_4;
    min_1 = min_2 = min_3 = min_4 = 1150.0;
    param_t channel1_max = param_find("RC1_MAX");
    param_t channel2_max = param_find("RC2_MAX");
    param_t channel3_max = param_find("RC3_MAX");
    param_t channel3_trim = param_find("RC3_TRIM");
    param_t channel4_max = param_find("RC4_MAX");
    param_t channel1_min = param_find("RC1_MIN");
    param_t channel2_min = param_find("RC2_MIN");
    param_t channel3_min = param_find("RC3_MIN");
    param_t channel4_min = param_find("RC4_MIN");

    set_command_param(&msg_data->command_data, 223, 1, 0, 0, 0, 0, 0, 0); //VEHICLE_CMD_DO_RC_CALIBRATION
    publish_commander_pd(msg_pd, msg_data);
    usleep(1000000);

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
    param_set(channel3_trim, &max_3);
    param_set(channel4_max, &max_4);
    param_set(channel1_min, &min_1);
    param_set(channel2_min, &min_2);
    param_set(channel3_min, &min_3);
    param_set(channel4_min, &min_4);
    orb_unsubscribe(input_rc_fd);

    set_command_param(&msg_data->command_data, 223, 0, 0, 0, 0, 0, 0, 0); //VEHICLE_CMD_DO_RC_CALIBRATION
    publish_commander_pd(msg_pd, msg_data);
}

void set_rc_channel_mid(void){
    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s value;
    float paramf = 1500.0;
    orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
    param_t rc1_mid_hd = param_find("RC1_TRIM");
    param_t rc2_mid_hd = param_find("RC2_TRIM");
    //param_t rc3_mid_hd = param_find("RC3_TRIM");
    param_t rc4_mid_hd = param_find("RC4_TRIM");

    paramf = value.values[0];
    param_set(rc1_mid_hd, &paramf);
    paramf = value.values[1];
    param_set(rc2_mid_hd, &paramf);
    //paramf = value.values[2];
    //param_set(rc3_mid_hd, &paramf);
    paramf = value.values[3];
    param_set(rc4_mid_hd, &paramf);
    orb_unsubscribe(input_rc_fd);
}

//bool check_mid(struct virtual_stick_s *vs_sp)
//{
//    int input_rc_fd =orb_subscribe(ORB_ID(input_rc));
//    struct input_rc_s value ={};
//    orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
//    bool rc_lost;
//    rc_lost = value.rc_failsafe || value.rc_lost || (value.timestamp == 0);
//    vs_sp->rc_signal_lost = rc_lost;
//    bool check_mid_pos;
//    check_mid_pos = (abs(value.values[0] -1500) < 50) && (abs(value.values[1] -1500) < 50)
//                             && (abs(value.values[2] -1500) < 50) &&(abs(value.values[3] -1500) < 50);
//    orb_unsubscribe(input_rc_fd);
//    printf("rc_lost is %d, check_mid_pos is %d\n", rc_lost, check_mid_pos);
//    return check_mid_pos || rc_lost;
//}

void publish_dg_mission_pd(MSG_orb_pub *msg_pd, struct dg_mission_s *mission_item)
{
    if (msg_pd->dg_mission_pd != NULL) {
        orb_publish(ORB_ID(dg_mission), msg_pd->dg_mission_pd, mission_item);
        printf("Passing 2_1\n");
    } else {
        msg_pd->dg_mission_pd = orb_advertise(ORB_ID(dg_mission), mission_item);
        printf("Passing 2_2\n");
    }
}

void wp_save(MSG_orb_pub *msg_pd){
    struct dg_mission_s mission_item = {};

    mission_item.timestamp = hrt_absolute_time();
    mission_item.msg_type = 0; //DG_MISSION_MSG
    mission_item.frame = 3; //MAV_FRAME_GLOBAL_RELATIVE_ALT
    mission_item.seq = wp_data.push->waypoint_seq -1;
    mission_item.current = mission_item.seq == 0 ? 1: 0;
    mission_item.mission_type = 0; //MAV_MISSION_TYPE_MISSION
    mission_item.autocontinue = wp_data.push->waypoint_seq < wp_data.total_num ? 1: 0;
    mission_item.target_system =1;
    mission_item.target_component =1;

    mission_item.x = (float)(((double)wp_data.push->lat) * 1e-7);
    mission_item.y = (float)(((double)wp_data.push->lon) * 1e-7);
    mission_item.z = ((float_t)wp_data.push->alt) / 10.0;
    mission_item.command = NAV_CMD_WAYPOINT;
    mission_item.param1 = ((float_t)wp_data.push->loiter_time);
    mission_item.param2 = -1;
    mission_item.param3 = 0;
    //mission_item.param3 = wp_data.push->photo_set > 0 ? (float)wp_data.push->photo_dis : 0;
    mission_item.param4 = NAN;
    //mission_item.param4 = wrap_2pi(math::radians( wp_data.push->yaw));
    mission_item.cruise_speed = ((float_t)wp_data.push->cruise_speed) /10.0;

    publish_dg_mission_pd(msg_pd, &mission_item);
}

void publish_vs_pd(MSG_orb_pub *msg_pd, struct virtual_stick_s *vs_sp)
{
    if (msg_pd->virtual_stick_pd != NULL){
            orb_publish(ORB_ID(virtual_stick), msg_pd->virtual_stick_pd, vs_sp);
            printf("Passing 2_1\n");
    }
    else{
            msg_pd->virtual_stick_pd = orb_advertise(ORB_ID(virtual_stick), vs_sp);
            printf("Passing 2_2\n");
    }
}

void msg_orb_param_pro(const uint8_t *buffer, MSG_orb_pub *msg_pd, MSG_orb_data *msg_data,
                                    MSG_param_hd msg_hd, MSG_type msg_type)
{
    int paramd;
    float paramf;
    struct dg_mission_s mission_item = {};

    switch (msg_type.name) {
    case MSG_NAME_WIFI:
        switch (msg_type.command) {
        case WIFI_COMM_WAYPOINT:
            set_command_param(&msg_data->command_data, 192, -1, 1, 0, NAN,
                              ((double_t)*(int32_t*)((uint32_t)buffer + 6)) * 1e-7,
                              ((double_t)*(int32_t*)((uint32_t)buffer + 10))* 1e-7,
                              (msg_data->global_position_data.alt));

//            set_command_param(&msg_data->command_data, 192, -1, 1, 0, NAN,
//                              lat_save,
//                              lon_save,
//                              (msg_data->global_position_data.alt));
//            if(lat_save != 0 && lon_save !=0)

            publish_commander_pd(msg_pd, msg_data);
            printf("lat is %.7f, lon is %.7f\n", ((double_t)*(int32_t*)((uint32_t)buffer + 6)) * 1e-7,
                   ((double_t)*(int32_t*)((uint32_t)buffer + 10))* 1e-7);
            printf("Passing waypoint\n");
            break;
        case WIFI_COMM_AUTO_LAND:
            set_command_param(&msg_data->command_data, 21, 0, 0, 0, NAN,
                              (msg_data->global_position_data.lat),
                              (msg_data->global_position_data.lon),
                              0);
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing land\n");
            break;
        case WIFI_COMM_AUTO_TAKEOFF:
            set_command_param(&msg_data->command_data, 22, 0, 0, 0, NAN,
                              (msg_data->global_position_data.lat),
                              (msg_data->global_position_data.lon),
                              (msg_data->global_position_data.alt + 5.0));
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing takeoff\n");
            break;
        case WIFI_COMM_WP_UPLOAD:
            printf("Start upload\n");
//            if (*(uint16_t*)((uint32_t)buffer + 6) > wp_data.total_num){
//                printf("Wrong waypoints sequece\n");
//            }else
            {
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
                wp_save(msg_pd);
                //if (wp_data.num == wp_data.total_num) printf("All waypoints is upload\n");
                //else wp_data.push = &wp_data.setd[wp_data.num];
                printf("Passing wy_upload, num is %d %d\n", wp_data.num, (wp_data.push - wp_data.setd +1));
                //response wp pushed
                setd_pack_send();
            }
            break;
        case WIFI_COMM_WP_UPLOAD_NUM:
            wp_data.total_num = *(uint16_t*)((uint32_t)buffer + 6);
            printf("Wp total_num is %d\n", wp_data.total_num);
            mission_item.timestamp = hrt_absolute_time();
            mission_item.msg_type = 1; //DG_MISSION_COUNT_MSG
            mission_item.mission_type = 0; //MAV_MISSION_TYPE_MISSION
            mission_item.target_system =1;
            mission_item.target_component =1;
            mission_item.count = wp_data.total_num;
            publish_dg_mission_pd(msg_pd, &mission_item);
            usleep(50000);
            printf("Passing wp_upload_num\n");
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
            mission_item.timestamp = hrt_absolute_time();
            mission_item.msg_type = 2; //DG_MISSION_SET_CURRENT
            mission_item.target_system =1;
            mission_item.target_component =1;
            mission_item.seq = (uint16_t)buffer[7] + ((uint16_t) buffer[8]<<8) -1;
            publish_dg_mission_pd(msg_pd, &mission_item);
            printf("Passing wp_chage\n");
            break;
        case WIFI_COMM_PARAM_GET:
            msg_pack_send(*msg_data, msg_pd);
            msg_param_saved_get(msg_hd);
            printf("Passing param_get\n");
            break;
        case WIFI_COMM_MAG_CALI:
            set_command_param(&msg_data->command_data, 241, 0, 1, 0, 0, 0, 0, 0);//CMD_PREFLIGHT_CALIBRATION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing mag_calibration\n");
            break;
        case WIFI_COMM_HIGHT_CHANGE:
            usleep(0);
            int home_position_fd = orb_subscribe(ORB_ID(home_position));
            struct home_position_s home_position_data = {};
            orb_copy(ORB_ID(home_position), home_position_fd, &home_position_data);
            set_command_param(&msg_data->command_data, 192, -1, 1, 0, NAN,
                              (msg_data->global_position_data.lat),
                              (msg_data->global_position_data.lon),
                              (home_position_data.alt + (float_t)*(int16_t*)((uint32_t)buffer + 7)/10.0));
            publish_commander_pd(msg_pd, msg_data);
            orb_unsubscribe(home_position_fd);
            printf("Passing hight_change, hight is %.4f\n", (float_t)*(int16_t*)((uint32_t)buffer + 7)/10.0);
            break;
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
            set_command_param(&msg_data->command_data, 176, 189, 4, 4, 0, 0, 0, 0);//VEHICLE_SET_MODE :MISSION
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing atuo_on\n");
            break;
        case WIFI_COMM_AUTO_FLIGHT_OFF:
            set_command_param(&msg_data->command_data, 176, 189, 4, 3, 0, 0, 0, 0); //VEHICLE_SET_MODE :LOITER
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing auto_off\n");
            break;
        case WIFI_COMM_GET_MID:
            set_rc_channel_mid();
            printf("Passing param_set get_mid\n");
            //response
            docap_pack_send(0);
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
            set_rc_channel_max_min(msg_pd, msg_data);
            printf("Set_rc_channel_limit Finish\n");
            //response
            docap_pack_send(1);
            break;
        case WIFI_COMM_POS_SAVE:
            lat_save = msg_data->global_position_data.lat;
            lon_save = msg_data->global_position_data.lon;
            break;
        case WIFI_COMM_REBOOT:
            if (msg_data->arm_data.armed == false){
                set_command_param(&msg_data->command_data, 246,
                                                3, 0, 0, 0, 0, 0, 0); //VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                publish_commander_pd(msg_pd, msg_data);
            }
            break;
        default:
            break;
        }
        break;

    case MSG_NAME_IWFI:        
        usleep(0);
        struct virtual_stick_s vs_sp ={};
        vs_sp.timestamp = hrt_absolute_time();
        vs_sp.r = ((float_t)(((uint16_t) buffer[5]<<8) + buffer [6])/1000.0 - 1.5)*2.0;
        vs_sp.y = ((float_t)(((uint16_t) buffer[7]<<8) + buffer [8])/1000.0 - 1.5)*2.0;
        vs_sp.x = ((float_t)(((uint16_t) buffer[9]<<8) + buffer [10])/1000.0 - 1.5)*2.0;
        vs_sp.z = 2.0 - (float_t)(((uint16_t) buffer[11]<<8) + buffer [12])/1000.0;
        //printf("Passing iwfi_pack, x = %.3f y = %.3f z = %.3f r = %.3f\n", vs_sp.x, vs_sp.y, vs_sp.z, vs_sp.r);
        //if ((msg_data->status_data.nav_state == 2 || msg_data->status_data.nav_state == 5) && check_mid(&vs_sp))
        if (msg_data->status_data.nav_state >=2&& msg_data->status_data.nav_state <=5)
            vs_sp.vs_enable = 1;
        else vs_sp.vs_enable = 0;
//        int input_rc_fd =orb_subscribe(ORB_ID(input_rc));
//        struct input_rc_s value = {};
//        orb_copy(ORB_ID(input_rc), input_rc_fd, &value);
//        vs_sp.rc_signal_lost = value.rc_failsafe || value.rc_lost || (value.timestamp == 0);
        vs_sp.rc_signal_lost = msg_data->status_data.rc_signal_lost;
         printf("timestamp is %lld\n", vs_sp.timestamp/1000);
        //printf("vs_enable = %d nav_state = %d\n", vs_sp.vs_enable, msg_data->status_data.nav_state);
         publish_vs_pd(msg_pd, &vs_sp);
        break;

    case MSG_NAME_YFWI:
        switch (msg_type.command) {
        case YFWI_COMM_YAW_FORCE:
            if (buffer[8] == 0) {
                paramd = 3;
                param_set(msg_hd.yaw_force_hd, &paramd);
            }
            else {
                paramd= 0;
                param_set(msg_hd.yaw_force_hd, &paramd);
            }
            printf("Passing yfwi_yaw_force\n");
            break;
        case YFWI_COMM_RETURN:
            set_command_param(&msg_data->command_data, 20, 0, 0, 0, 0, 0, 0, 0);
                                           //VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing yfwi_return\n");
            break;
        default:
            break;
        }
        break;

    case MSG_NAME_EXYF:
        switch (msg_type.command) {
        case EXYF_COMM_LOITER_YAW:
            if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
                set_command_param(&msg_data->command_data, 17, 0, 0, 0,
                                  (float_t)*(int32_t*)((uint32_t)buffer + 9),
                                  (msg_data->global_position_data.lat),
                                  (msg_data->global_position_data.lon),
                                  (msg_data->global_position_data.alt));
            }
            publish_commander_pd(msg_pd, msg_data);
            printf("Passing loiter_yaw\n");
            break;
        case EXYF_COMM_LOITER_MOVE:
//             if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
//                if (buffer[11] & 0xf0){
//                    //double lat, lon;
//                    float x, y;
//                    if (buffer[11] & 0x30){
//                        y = msg_data->local_position_sp_data.y +  (buffer[11] & 0x10 ? -1:1)*((float_t)*(uint16_t*)((uint32_t)buffer + 9)/10.0);
//                        x = msg_data->local_position_sp_data.x;
//                    }
//                    else {
//                        x = msg_data->local_position_sp_data.x+  (buffer[11] & 0x40 ? -1:1)*((float_t)*(uint16_t*)((uint32_t)buffer + 9)/10.0);
//                        y = msg_data->local_position_sp_data.y;
//                    }

//                    //if (! map_projection_global_reproject(x, y, &lat, &lon)){
//    //                    msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
//    //                    msg_data->command_data.param5 = (float_t)lat;
//    //                    msg_data->command_data.param6 = (float_t)lon;
//    //                    msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
//    //                    msg_data->command_data.param1 = 0.0;
//    //                    msg_data->command_data.param2 = 0.0;
//    //                    msg_data->command_data.param3 = 0.0;
//    //                    msg_data->command_data.param4 = 0.0;
//                    //}

//                    msg_data->local_position_sp_data.x = x;
//                   msg_data->local_position_sp_data.y = y;
//                }
//             }
//             publish_local_position_sp_pd(msg_pd, msg_data);
//             printf("Passing loiter_move\n");
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
            //paramf = (float_t)*(uint16_t*)((uint32_t)buffer + 9);
            usleep(0);
            int home_position_fd = orb_subscribe(ORB_ID(home_position));
            struct home_position_s home_position_data = {};
            orb_copy(ORB_ID(home_position), home_position_fd, &home_position_data);
            memcpy(&paramf , &buffer[9], sizeof(float));
            set_command_param(&msg_data->command_data, 17, 0, 0, 0, 0,
                              (msg_data->global_position_data.lat),
                              (msg_data->global_position_data.lon),
                              (home_position_data.alt+ paramf /10.0));
            memcpy(&paramf , &buffer[11], sizeof(float));
            param_set(msg_hd.up_vel_max_hd, &paramf);
            param_set(msg_hd.dn_vel_max_hd, &paramf);
            publish_commander_pd(msg_pd, msg_data);
            orb_unsubscribe(home_position_fd);
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

int find_r_type( uint8_t *buffer, MSG_orb_data *msg_data,  MSG_orb_pub *msg_pd,
                        MSG_param_hd msg_hd)
{
    MSG_type msg_type;
    memset(&msg_type, 0, sizeof(msg_type));
    //int nread = 0;

    char *name = "$WIFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing WIFI\n");
        //nread +=read_to_buff(buffer, 5, 30);
        //if (nread == 25)
        {

            msg_type.name =MSG_NAME_WIFI;
            msg_type.command = buffer[5];

            printf("Sum check is %d, %d\n", buffer[29], calculate_sum_check(buffer, 30));
            //if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
            if (check_command_repeat(buffer, msg_type) && (buffer[29] == 0x3f || buffer[29] == calculate_sum_check(buffer, 30)))
            {
                printf("Passing check\n");
                msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                return 30;
            }
            else return -1;
        }

    }

    name = "$YFWI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing YFWI\n");
        uint8_t buflen;
        //nread +=read_to_buff(buffer, 5, 6);
        //if (nread < 1) return nread;
        buflen = buffer[5];
        printf("buffer len is %d\n", buflen);
        //nread +=read_to_buff(buffer, 6, buflen);
        //if (nread == (buflen -5))
        {
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
                        YFPA_param yfpa_param;
                         memset(&yfpa_param, 0, sizeof(YFPA_param));
                        yfpa_param_pack(&yfpa_param, msg_hd);
                        memcpy(param_saved, &yfpa_param, sizeof(YFPA_param));
                        uint16_t crc = check_crc(param_saved, 62);
                        param_saved[60] = (uint8_t)(crc & 0x00ff);
                        param_saved[61] = (uint8_t)((crc & 0xff00)>>8);
                        write(uart_read, param_saved, sizeof(YFPA_param));
                        printf("Response Sended\n");
                    }
                }
                else {
                    msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                }
                return buflen;
            }
            else return -1;
        }
    }

    name = "$IWFI";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing IWFI\n");
        //nread +=read_to_buff(buffer, 5, 30);
        //if (nread == 25)
         {
            msg_type.name =MSG_NAME_IWFI;
            //if(check_command_repeat(buffer, msg_type) && buffer[29] == calculate_sum_check(buffer, 30))
            if (check_command_repeat(buffer, msg_type) && (buffer[29] == 0x3f || buffer[29] == calculate_sum_check(buffer, 30)))
             {
                 printf("Passing check\n");
                 msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                 return 30;
             }
            else return -1;
         }
    }

    name = "$HFMR";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing HFMR\n");
        //nread +=read_to_buff(buffer, 5, 30);
        //if (nread == 25)
        {
            msg_type.name =MSG_NAME_HFMR;
            if(buffer[29] == calculate_sum_check(buffer, 30))
             //if (buffer[29] == 0x3f)
             {
                param_reset_all();
                return 30;
             }
            else return -1;
        }
    }

    name = "$EXYF";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXYF\n");
        uint8_t buflen;
        //nread +=read_to_buff(buffer, 5, 7);
        //if (nread < 2) return nread;
        //if (!read_to_buff(buffer, 5, 7)) return;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        printf("Buflen is %d\n", buflen);
        //nread +=read_to_buff(buffer, 7, buflen);
        //if (nread == (buflen - 5))
        //if (read_to_buff(buffer, 7, buflen))
        {
            msg_type.name =MSG_NAME_EXYF;
            msg_type.command = buffer[7];
            uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
            //if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
            if (check_command_repeat(buffer, msg_type) && buffer[buflen -1] == 0x3f)
             {
                //printf("Check passed\n");
                msg_orb_param_pro(buffer, msg_pd, msg_data, msg_hd, msg_type);
                exyf_response_pack(msg_type, msg_hd);
                return buflen;
             }
            else return -1;
        }
    }

    name = "$EXEX";
    if (compare_buffer_n(buffer, (uint8_t*)name, 5))
    {
        printf("Passing EXEF\n");
        uint8_t buflen;
        //nread +=read_to_buff(buffer, 5, 7);
        //if (nread < 2) return nread;
        //if (!read_to_buff(buffer, 5, 7)) return;
        buflen = (uint16_t)buffer[5] + ((uint16_t)buffer[6]<<8);
        //nread +=read_to_buff(buffer, 7, buflen);
        //if (nread == (buflen - 5))
        //if (read_to_buff(buffer, 7, buflen))
        {
        msg_type.name =MSG_NAME_EXEX;
        msg_type.command = buffer[7];
        uint16_t  crc_receive = (uint16_t)buffer[buflen -2] + ((uint16_t)buffer[buflen -1] << 8);
        if (check_command_repeat(buffer, msg_type) && crc_receive == check_crc(buffer, buflen))
        //if (check_command_repeat(buffer, msg_type) && buffer[buflen + 8] == 0x3f)
         {
            msg_orb_param_pro(buffer ,msg_pd, msg_data, msg_hd, msg_type);
            return buflen;
         }
        else return -1;
        }
    }
    return -1;
}

