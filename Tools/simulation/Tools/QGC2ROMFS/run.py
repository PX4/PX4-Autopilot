#!/usr/bin/env python3

import argparse
from pathlib import Path
import os
import shutil
import subprocess
import time

import qgc_to_gz
import clean
import file_checker

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--params", required=True, help="Path to input params to be converted from QGroundControl (.params).")
    parser.add_argument("--romfs", required=True, help="Path to ROMFS file used as a base to which other params will be apended.")
    parser.add_argument("--PX4", required=True, help="Path to PX4 directory.")
    parser.add_argument("--sim", required=True, help="Simulator to be used (gz, gazebo-classic, jsbsim, sihsim, flightgear)")
    inputs = parser.parse_args()

    params_path = Path(inputs.params)
    romfs_path = Path(inputs.romfs)
    [final_romfs_file, final_romfs_path] = qgc_to_gz.main(params_path, romfs_path, inputs.sim)

    params_file = os.path.basename(params_path)
    vehicle_name = params_file.split('.')[0]
    newpath = vehicle_name
    #Some parameters are not defined in Gazebo so a simulation is ran to find this [ERRORS] and store them in a file 
    px4_path = Path(inputs.PX4)
    try:
        subprocess.run(["bash", "./make_px4_script.sh", vehicle_name, px4_path], check=True)
        print("Script executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error while executing the script: {e}")
        return -1
    
    #when the subprocess ends it creates a "end file"
    done_file = "./px4_done"
    # Wait for PX4 to complete
    while not os.path.exists(done_file):
        time.sleep(1)
    os.remove(done_file)

    errors_file = f"{newpath}/errors.txt"
    if Path(f"errors.txt").is_file():
        shutil.move("errors.txt", f"{newpath}/")
    else:
        errors_file = None
    
    #clean will comment out any parameters that were not recognized by Gazebo as well as parameters that are not compatible with simulated sensors
    clean.main(f"{newpath}/{final_romfs_file}", f"{newpath}/errors.txt")

    #pass the final version of the ROMFS file to the PX4 ROMFS folder
    shutil.copy(f"{newpath}/{final_romfs_file}", final_romfs_path)

    print(f"\033[92m[INFO]\033[0m: Generated the romfs file")

    file_checker.main(romfs_path)

if __name__ == '__main__':
    main()
