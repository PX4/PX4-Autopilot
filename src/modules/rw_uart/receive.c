#include "rw_uart.h"
#include "rw_uart_define.h"

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

void wifi_pack(const uint8_t *buffer, MSG_orb_data *msg_data, MSG_type msg_type){
    //void *data;
    switch (msg_type.command) {
    case WIFI_COMM_WAYPOINT:
        msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
        //data = ((int32_t) buffer[6]) + ((int32_t) buffer[7]<<8) +((int32_t) buffer[8]<<16) + ((int32_t)buffer[9]<<24);
        //data =(void *)((uint32_t)buffer + 6);
        msg_data->command_data.param5 = ((float_t)*(int32_t*)((uint32_t)buffer + 6)) * 1e-7;
        printf("lat: %d\n", *(int32_t*)((uint32_t)buffer + 6));
        msg_data->command_data.param6 = ((float_t)*(int32_t*)((uint32_t)buffer + 10))* 1e-7;
        printf("lon: %d\n", *(int32_t*)((uint32_t)buffer + 10));
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
        printf("Passing waypoint\n");
        msg_data->command_data.param1 = 0.0;
        msg_data->command_data.param2 = 0.0;
        msg_data->command_data.param3 = 0.0;
        msg_data->command_data.param4 = 0.0;
        break;
    case WIFI_COMM_AUTO_LAND:
        msg_data->command_data.command = 21; //VEHICLE_CMD_NAV_LAND
        msg_data->command_data.param4 = 0.0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = 0.0;
        printf("Passing land\n");
        break;
    case WIFI_COMM_AUTO_TAKEOFF:
        msg_data->command_data.command = 22; //VEHICLE_CMD_NAV_TAKEOFF
        msg_data->command_data.param1 = 0.0;
        msg_data->command_data.param4 = 0.0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0 +100.0;
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
    case WIFI_COMM_GYRO_CLEAR:
        msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
        msg_data->command_data.param1 = 1;
        msg_data->command_data.param2 = 0;
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = 0;
        msg_data->command_data.param5 = 0;
        msg_data->command_data.param6 = 0;
        msg_data->command_data.param7 = 0;
        printf("Passing gyro_calibration\n");
        break;
    case WIFI_COMM_WP_CHAGE:
        msg_data->command_data.command = 177; //CMD_DO_JUMP
        msg_data->command_data.param1 = (uint16_t)buffer[7] + ((uint16_t) buffer[8]<<8);
        msg_data->command_data.param2 = 0;
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
        printf("Passing mag_calibration\n");
        break;
    case WIFI_COMM_HIGHT_CHANGE:
        msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = ((float_t)*(int16_t*)((uint32_t)buffer + 7))/10.0;
        msg_data->command_data.param1 = 0.0;
        msg_data->command_data.param2 = 0.0;
        msg_data->command_data.param3 = 0.0;
        msg_data->command_data.param4 = 0.0;
        printf("Passing hight_change\n");
        break;
    case WIFI_COMM_RC_POS:
        msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
        msg_data->command_data.param1 = 0;
        msg_data->command_data.param2 = 0;
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = 1;
        msg_data->command_data.param5 = 0;
        msg_data->command_data.param6 = 0;
        msg_data->command_data.param7 = 0;
        printf("Passing rc_cali\n");
        break;
    case WIFI_COMM_ESC_CALI_ON:
        msg_data->command_data.command = 241; //CMD_PREFLIGHT_CALIBRATION
        msg_data->command_data.param1 = 0;
        msg_data->command_data.param2 = 0;
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = 0;
        msg_data->command_data.param5 = 0;
        msg_data->command_data.param6 = 0;
        msg_data->command_data.param7 = 1;
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
        printf("Passing cali_off\n");
        break;
    case WIFI_COMM_AUTO_FLIGHT_ON:
        msg_data->command_data.command = 300; //CMD_MISSION_START
        msg_data->command_data.param1 = wp_data.setd[0].waypoint_seq;
        msg_data->command_data.param2 = wp_data.push->waypoint_seq;
        printf("Passing atuo_on\n");
        break;
    case WIFI_COMM_AUTO_FLIGHT_OFF:
        msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = 0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
        printf("Passing auto_off\n");
        break;
    case WIFI_COMM_DISARMED:
        msg_data->arm_data.lockdown = true;
        msg_data->arm_data.armed = false;
        msg_data->status_data.arming_state = 0; //ARMING_STATE_INIT
        msg_data->control_mode_data.flag_armed = false;
        printf("Passing disarm\n");
        break;
    case WIFI_COMM_ARMED:
        //msg_data->command_data.command = 400; //CMD_COMPONENT_ARM_DISARM
        //msg_data->command_data.param1 = 1;
        msg_data->arm_data.lockdown = false;
        msg_data->arm_data.armed = true;
        msg_data->status_data.arming_state = 2; //ARMING_STATE_ARMED
        msg_data->control_mode_data.flag_armed = true;
        printf("Passing arm\n");
        break;
    default:
        break;
    }
}

bool change_param (MSG_param_hd msg_hd, uint8_t data, int i){
    bool changed = true;
    float_t paramf;
    int paramd;
    switch (i) {
    case 8:
        paramf = (float_t)data / 21.25;
        param_set(msg_hd.roll_p_hd, &paramf);
        //printf("Passing change\n");
        break;
//    case 11:
//        paramf = (float_t)data / 21.25;
//        param_set(msg_hd.pitch_p_hd, &paramf);
//        break;
//    case 14:
//        paramf = (float_t)data / 51.0;
//        param_set(msg_hd.yaw_p_hd, &paramf);
//        break;
//    case 17:
//        paramf = (float_t)data / 170.0;
//        param_set(msg_hd.z_p_hd, &paramf);
//        break;
//    case 21:
//        paramf = (float_t)data / 34.0 + 0.5;
//        param_set(msg_hd.up_vel_max_hd, &paramf);
//        break;
//    case 22:
//        paramf = (float_t)data / 15.0 + 3.0;
//        param_set(msg_hd.xy_vel_max_hd, &paramf);
//        break;
//    case 23:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.roll_rate_hd, &paramf);
//        break;
//    case 24:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.pitch_rate_hd, &paramf);
//        break;
//    case 25:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.yaw_rate_hd, &paramf);
//        break;
//    case 26:
//        paramf = (float_t)data / 19.62 + 2.0;
//        param_set(msg_hd.acc_up_max_hd, &paramf);
//        break;
//    case 28:
//        paramf = (float_t)data / 0.6375;
//        param_set(msg_hd.yaw_max_hd, &paramf);
//        break;
//    case 29:
//        paramf = (float_t)data / 2.84;
//        param_set(msg_hd.roll_max_hd, &paramf);
//        break;
//    case 30:
//        paramf = (float_t)data / 1.60 + 20.0;
//        param_set(msg_hd.pitch_max_hd, &paramf);
//        break;
//    case 33:
//        paramf = (float_t)data / 19.62 + 2.0;
//        param_set(msg_hd.acc_hor_max_hd, &paramf);
//        break;
//    case 38:
//        paramd = (int)data;
//        param_set(msg_hd.mav_type_hd, &paramd);
//        break;
//    case 39:
//        paramd = (int)data;
//        param_set(msg_hd.battery_n_cells_hd, &paramd);
//        break;
//    case 40:
//        paramf = (float_t)data / 910.8 + 0.12;
//        param_set(msg_hd.battery_warn_hd, &paramf);
//        break;
//    case 48:
//        paramd = (int)data;
//        param_set(msg_hd.battery_warn_hd, &paramd);
//        break;
//    case 49:
//        paramd = (int)data;
//        param_set(msg_hd.rc_lost_act_hd, &paramd);
//        break;
//    case 59:
//        paramf = (float_t)data / 63.75;
//        param_set(msg_hd.dn_vel_max_hd, &paramf);
//        break;
    default:
        changed = false;
        break;
    }
    printf("Passing change, changed is %d, i is %d\n", changed, i);
    return changed;
}

bool yfwi_param_set(const uint8_t *buffer, MSG_param_hd msg_hd){
    bool changed = false;
    uint16_t  dist_max, dist_max_saved;

    dist_max = ((uint16_t) buffer[32]<< 8) + (uint16_t)buffer[31];
    dist_max_saved = ((uint16_t) param_saved[32]<< 8) + (uint16_t)param_saved[31];
    if(dist_max != dist_max_saved) {
        float_t paramf = (float_t)dist_max / 6.5535;
        param_set(msg_hd.higt_max_hd, &paramf);
        changed = true;
    }

    dist_max = ((uint16_t) buffer[35]<< 8) + (uint16_t)buffer[34];
    dist_max_saved = ((uint16_t) param_saved[35]<< 8) + (uint16_t)param_saved[34];
    if(dist_max != dist_max_saved) {
        float_t paramf = (float_t)dist_max / 6.5535;
        param_set(msg_hd.dist_max_hd, &paramf);
        changed = true;
        printf("dist_max changed\n");
    }

    for (int i = 8; i < 60; i++) {
        printf("i : %d, buffer[i] : %x, param_saved[i]: %x\n", i, buffer[i], param_saved[i]);
        if (buffer[i] != param_saved[i])
        {
            changed |= change_param(msg_hd, buffer[i], i);
            printf("Passing change, changed is %d\n", changed);
        }
    }
    return changed;
}

void iwfi_pack(const uint8_t *buffer, MSG_orb_data *msg_data){
    msg_data->manual_data.r = ((float_t)(((uint16_t) buffer[5]<<8) + buffer [6])/65535.0 - 0.5)*2.0;
    msg_data->manual_data.y = ((float_t)(((uint16_t) buffer[7]<<8) + buffer [8])/65535.0 - 0.5)*2.0;
    msg_data->manual_data.x = ((float_t)(((uint16_t) buffer[9]<<8) + buffer [10])/65535.0 - 0.5)*2.0;
    msg_data->manual_data.z = (float_t)(((uint16_t) buffer[11]<<8) + buffer [12])/65535.0;
    printf("Passing iwfi_pack\n");
}

void yfwi_pack(const uint8_t *buffer, MSG_type msg_type, MSG_param_hd msg_hd){
    int paramd;
    switch (msg_type.command) {
    case YFWI_COMM_YAW_FORCE:
        if (buffer[8] == 1) {
            paramd = 1;
            param_set(msg_hd.yaw_force_hd, &paramd);
        }
        else {
            paramd= 0;
            param_set(msg_hd.yaw_force_hd, &paramd);
        }
        printf("Passing yfwi_pack\n");
        break;
    default:
        break;
    }
}

void exyf_pack(const uint8_t *buffer, MSG_orb_data *msg_data, MSG_type msg_type, MSG_param_hd msg_hd){
    int paramd;
    switch (msg_type.command) {
    case EXYF_COMM_LOITER_YAW:
        if (msg_data->status_data.nav_state == 4){ //NAVIGATION_STATE_AUTO_LOITER
        msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = (float_t)*(int32_t*)((uint32_t)buffer + 9);
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
        }
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
//                    msg_data->command_data.param6 = (float_t)lon;;
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
    default:
        break;
    }
}

void exex_pack(const uint8_t *buffer, MSG_orb_data *msg_data, MSG_type msg_type, MSG_param_hd msg_hd){
    float paramf;
    switch (msg_type.command) {
    case EXEX_COMM_HIGHT_CHANGE:
        paramf = (float)*(uint16_t*)((uint32_t)buffer + 9);
        msg_data->command_data.command = 17; //CMD_NAV_LOITER_UNLIM
        msg_data->command_data.param3 = 0;
        msg_data->command_data.param4 = 0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)* 1e-7;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)* 1e-7;
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0 + paramf /10.0;
        paramf = *(float_t*)((uint32_t)buffer + 11);
        param_set(msg_hd.up_vel_max_hd, &paramf);
        param_set(msg_hd.dn_vel_max_hd, &paramf);
        break;
    default:
        break;
    }
}
