#!/usr/bin/env python3

from pathlib import Path
from collections import Counter

def comment_errors(errors_file, param_to_comment):
    with open(errors_file, 'r') as ef:
        for line in ef:
            parts = line.strip().split(' ')
            if len(parts) > 3:
                param_to_comment.add(parts[3])  

def comment_if_simulated(param_name_set, param_to_comment):
    gps_params_for_real = {'CAL_ACC0_ID', 'CAL_ACC0_PRIO', 'CAL_ACC0_ROT', 'CAL_ACC0_XOFF', 
                           'CAL_ACC0_XSCALE', 'CAL_ACC0_YOFF', 'CAL_ACC0_YSCALE', 'CAL_ACC0_ZOFF', 
                           'CAL_ACC0_ZSCALE', 'CAL_ACC1_ID', 'CAL_ACC1_PRIO', 'CAL_ACC1_ROT', 
                           'CAL_ACC1_XOFF', 'CAL_ACC1_XSCALE', 'CAL_ACC1_YOFF', 'CAL_ACC1_YSCALE', 
                           'CAL_ACC1_ZOFF', 'CAL_ACC1_ZSCALE', 'CAL_ACC2_ID', 'CAL_ACC3_ID', 
                           'CAL_AIR_CMODEL', 'CAL_AIR_TUBED_MM', 'CAL_AIR_TUBELEN', 'CAL_BARO0_ID', 
                           'CAL_BARO1_ID', 'CAL_BARO2_ID', 'CAL_BARO3_ID', 'CAL_GYRO0_ID', 
                           'CAL_GYRO0_PRIO', 'CAL_GYRO0_ROT', 'CAL_GYRO0_XOFF', 'CAL_GYRO0_YOFF', 
                           'CAL_GYRO0_ZOFF', 'CAL_GYRO1_ID', 'CAL_GYRO1_PRIO', 'CAL_GYRO1_ROT', 
                           'CAL_GYRO1_XOFF', 'CAL_GYRO1_YOFF', 'CAL_GYRO1_ZOFF', 'CAL_GYRO2_ID', 
                           'CAL_GYRO3_ID'}

    mag_params_for_real = {'CAL_MAG0_ID', 'CAL_MAG0_PRIO', 'CAL_MAG0_ROT', 'CAL_MAG0_XCOMP', 
                           'CAL_MAG0_XODIAG', 'CAL_MAG0_XOFF', 'CAL_MAG0_XSCALE', 'CAL_MAG0_YCOMP', 
                           'CAL_MAG0_YODIAG', 'CAL_MAG0_YOFF', 'CAL_MAG0_YSCALE', 'CAL_MAG0_ZCOMP', 
                           'CAL_MAG0_ZODIAG', 'CAL_MAG0_ZOFF', 'CAL_MAG0_ZSCALE', 'CAL_MAG1_ID', 
                           'CAL_MAG1_PRIO', 'CAL_MAG1_ROT', 'CAL_MAG1_XCOMP', 'CAL_MAG1_XODIAG', 
                           'CAL_MAG1_XOFF', 'CAL_MAG1_XSCALE', 'CAL_MAG1_YCOMP', 'CAL_MAG1_YODIAG', 
                           'CAL_MAG1_YOFF', 'CAL_MAG1_YSCALE', 'CAL_MAG1_ZCOMP', 'CAL_MAG1_ZODIAG', 
                           'CAL_MAG1_ZOFF', 'CAL_MAG1_ZSCALE', 'CAL_MAG2_ID', 'CAL_MAG2_ROT', 
                           'CAL_MAG3_ID', 'CAL_MAG3_ROT', 'CAL_MAG_COMP_TYP', 'CAL_MAG_SIDES'}

    air_speed_params_for_real = {'ASPD_BETA_GATE', 'ASPD_BETA_NOISE', 'ASPD_DO_CHECKS', 'ASPD_FALLBACK_GW', 
                                'ASPD_FS_INNOV', 'ASPD_FS_INTEG', 'ASPD_FS_T_START', 'ASPD_FS_T_STOP', 
                                'ASPD_PRIMARY', 'ASPD_SCALE_1', 'ASPD_SCALE_2', 'ASPD_SCALE_3', 
                                'ASPD_SCALE_APPLY', 'ASPD_SCALE_NSD', 'ASPD_TAS_GATE', 'ASPD_TAS_NOISE', 
                                'ASPD_WERR_THR', 'ASPD_WIND_NSD'}

    if "SENS_EN_GPSSIM" in param_name_set:
        param_to_comment.update(gps_params_for_real)

    if "SENS_EN_MAGSIM" in param_name_set:
        param_to_comment.update(mag_params_for_real)

    if "SENS_EN_ARSPDSIM" in param_name_set:
        param_to_comment.update(air_speed_params_for_real)

def comment_real_servos(param_to_comment):
    real_params_for_servos = {'PWM_MAIN_FUNC1', 'PWM_MAIN_FUNC2', 'PWM_MAIN_FUNC3', 'PWM_MAIN_FUNC4', 
                              'PWM_MAIN_FUNC5', 'PWM_MAIN_FUNC6', 'PWM_MAIN_FUNC7', 'PWM_MAIN_FUNC8'}
    param_to_comment.update(real_params_for_servos)

def comment_break_checks(param_to_comment):
    real_params_for_servos = {'CBRK_BUZZER', 'CBRK_FLIGHTTERM', 'CBRK_IO_SAFETY', 'CBRK_SUPPLY_CHK', 
                              'CBRK_USB_CHK', 'CBRK_VTOLARMING'}
    param_to_comment.update(real_params_for_servos)

def main(romfs_path, error_path):
    param_name_set = set()
    param_to_comment = set()
    param_name_counter = Counter() 

    with open(romfs_path, 'r') as pf:
        lines = pf.readlines()

    for line in lines:
        if line.startswith('#'):
            continue

        parts = line.strip().split(' ')
        if len(parts) > 2 and parts[0] == "param":
            param_name_set.add(parts[2])

    if error_path is not None:
        comment_errors(error_path, param_to_comment)
    comment_if_simulated(param_name_set, param_to_comment)
    comment_real_servos(param_to_comment)
    comment_break_checks(param_to_comment)

    new_lines = []
    for line in lines:
        if line.startswith('#'):
            new_lines.append(line)
            continue

        parts = line.strip().split(' ')

        if "SENS_BOARD_ROT" in parts:
            print("\033[92m[INFO]\033[0m: changed SENS_BOARD_ROT to 0.0 since the board must be front facing in the simulation")
            new_lines.append("param set-default SENS_BOARD_ROT 0.0 #Change made: in simulation the Board is allways facing frontwards (YAW 0 deg)\n")
            continue

        if len(parts) > 2 and parts[0] == "param":
            param_name_counter[parts[2]] += 1  # Increment count for each occurrence

        if len(parts) > 2 and parts[0] == "param" and parts[2] in param_to_comment:
            new_lines.append(f"# {line}")  # Comment out the line
        else:
            new_lines.append(line)

    with open(romfs_path, 'w') as pf:
        pf.writelines(new_lines)

    print(f"\033[92m[INFO]\033[0m: removed erroneous parameters")

if __name__ == '__main__':
    main()
