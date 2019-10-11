#include "rw_uart.h"
#include "rw_uart_define.h"

int find_frame(uint8_t data){
    int paramd = 5001;
    switch (data) {
//        case 0:
//            paramd = 5001;
//            break;
    case 1:
        paramd = 4001;
        break;
    case 2:
        paramd = 7001;
        break;
    case 3:
        paramd = 6001;
        break;
    case 4:
        paramd = 9001;
        break;
    case 5:
        paramd = 8001;
        break;
    case 6:
        paramd = 11001;
        break;
    case 7:
        paramd = 14001;
        break;
    case 8:
        paramd = 12001;
        break;
    default:
        paramd = 5001;
        break;
    }
    return paramd;
}

bool change_param (MSG_param_hd msg_hd, uint8_t data, int i){
    bool changed = true;
    float_t paramf;
    int paramd;
    switch (i) {
    case 8:
        paramf = (float_t)data / 510.0;
        param_set(msg_hd.roll_p_hd, &paramf);
        //printf("Passing change\n");
        break;
    case 9:
        paramf = (float_t)data / 5100.0;
        param_set(msg_hd.roll_i_hd, &paramf);
        break;
    case 10:
        paramf = (float_t)data / 25500.0;
        param_set(msg_hd.roll_d_hd, &paramf);
        break;
    case 11:
        paramf = (float_t)data / 510.0;
        param_set(msg_hd.pitch_p_hd, &paramf);
        break;
    case 12:
        paramf = (float_t)data / 5100.0;
        param_set(msg_hd.pitch_i_hd, &paramf);
        break;
    case 13:
        paramf = (float_t)data / 25500.0;
        param_set(msg_hd.pitch_d_hd, &paramf);
        break;
    case 14:
        paramf = (float_t)data / 510.0;
        param_set(msg_hd.yaw_p_hd, &paramf);
        break;
    case 15:
        paramf = (float_t)data / 1275.0;
        param_set(msg_hd.yaw_i_hd, &paramf);
        break;
    case 16:
        paramf = (float_t)data / 25500.0;
        param_set(msg_hd.roll_d_hd, &paramf);
        break;
    case 17:
        paramf = (float_t)data / 170.0;
        param_set(msg_hd.z_p_hd, &paramf);
        break;
    case 21:
        paramf = (float_t)data / 34.0 + 0.5;
        param_set(msg_hd.up_vel_max_hd, &paramf);
        break;
    case 22:
        paramf = (float_t)data / 15.0 + 3.0;
        param_set(msg_hd.xy_vel_max_hd, &paramf);
        break;
    case 23:
        paramf = (float_t)data;
        param_set(msg_hd.roll_rate_hd, &paramf);
        break;
    case 24:
        paramf = (float_t)data;
        param_set(msg_hd.pitch_rate_hd, &paramf);
        break;
    case 25:
        paramf = (float_t)data;
        param_set(msg_hd.yaw_rate_hd, &paramf);
        break;
    case 26:
        paramf = (float_t)data / 19.62 + 2.0;
        param_set(msg_hd.acc_up_max_hd, &paramf);
        break;
    case 28:
        paramf = (float_t)data;
        param_set(msg_hd.yaw_max_hd, &paramf);
        break;
    case 29:
        paramf = (float_t)data / 2.84;
        param_set(msg_hd.roll_max_hd, &paramf);
        break;
    case 30:
        paramf = (float_t)data / 1.59 + 20.0;
        param_set(msg_hd.pitch_max_hd, &paramf);
        break;
    case 33:
        paramf = (float_t)data / 19.61 + 2.0;
        param_set(msg_hd.acc_hor_max_hd, &paramf);
        break;
    case 38:
        paramd = find_frame(data);
        param_set(msg_hd.mav_type_hd, &paramd);
        break;
    case 39:
        paramd = (int)data;
        param_set(msg_hd.battery_n_cells_hd, &paramd);
        break;
    case 40:
        paramf = ((float_t)((data & 0x0c)/4) + 1.2) * 0.05/0.55;
        //paramf = (float_t)data / 910.8 + 0.12;
        param_set(msg_hd.battery_warn_hd, &paramf);
        break;
    case 48:
        switch (data & 0x0f) {
        case 0x00:
            paramd = 3;
            break;
        case 0x06:
            paramd = 0;
            break;
        case 0x04:
            paramd = 1;
            break;
        case 0x01:
        case 0x02:
        case 0x05:
            paramd = 2;
            break;
        default:
            paramd = 0;
            break;
        }
        param_set(msg_hd.battery_fail_hd, &paramd);
        switch (data & 0x30) {
        case 0x00:
            paramd = 2;
            break;
        case 0x10:
            paramd = 3;
            break;
        default:
            paramd = 1;
            break;
        }
         param_set(msg_hd.rc_lost_act_hd, &paramd);
        break;
//    case 49:
//        paramd = (int)data;
//        param_set(msg_hd.rc_lost_act_hd, &paramd);
//        break;
    case 59:
        paramf = (float_t)data / 63.75;
        param_set(msg_hd.dn_vel_max_hd, &paramf);
        break;
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
        //if (buffer[i] != param_saved[i])
        if (abs(buffer[i] != param_saved[i]))
        {
            changed |= change_param(msg_hd, buffer[i], i);
            //printf("Passing change, changed is %d\n", changed);
        }
    }
    return changed;
}
