#!/usr/bin/env python3
from pathlib import Path
import os
import shutil

def process_params_file(params_file, output_file):
    """
    Reads the params_file, extracts the parameter name and value,
    and writes the formatted output to the initd_file.
    """
    with open(params_file, 'r') as pf, open(output_file, 'a') as of:
        for line in pf:
            if line[0] == '#':
                continue
            parts = line.strip().split('\t')
            if len(parts) < 2:
                continue
            param_name = parts[2]
            param_value = float(parts[3])
            of.write(f"param set-default {param_name} {param_value}\n")

def main(params_path, romfs_path, simulator):
    params_file = os.path.basename(params_path)
    newpath = params_file.split('.')[0]
    vehicle_name = newpath
    
    if not romfs_path.is_file():
        raise ValueError("Error: --romfs_path must be a valid file path.")

    airframes_path = ""
    romfs_path_parts = str(romfs_path).split("/")
    for i in range(len(romfs_path_parts)-1):
        airframes_path += romfs_path_parts[i] + '/'

    #check if the romfs file exist and if not create a new file with a incremented number
    final_romfs_file = ""
    biggest_numb = 22000
    for (_, _, files) in os.walk(airframes_path):
        for file in files:
            if "_gz_" not in file: #ignore non gazebo garden simulations
                continue
            file_parts = file.split("_gz_")
            file_name = file_parts[1]
            if int(file_parts[0]) > biggest_numb:
                biggest_numb = int(file_parts[0])
            if vehicle_name == file_name:
                final_romfs_file = file
        break
    #if the file did not exit previously then created and added it to the CmakeList
    if final_romfs_file == "":
        final_romfs_file = str(biggest_numb+1) + "_" + simulator + "_" + vehicle_name
        with open(airframes_path + "CMakeLists.txt", "r") as cf:
            lines = cf.readlines()

        for i in range(len(lines)):
            if lines[i][0] == ")":
                lines.insert(i-1, '\t' + final_romfs_file +'\n') 
                break
        
        # Write back to the file
        with open(airframes_path + "CMakeLists.txt", "w") as cf:
            cf.writelines(lines)

    #check for files of a previous code execution and remove them
    if Path(f"{newpath}/{params_file}").is_file(): #param file
        shutil.move(f"{newpath}/{params_file}", f"./{params_file}")
    if Path(f"{newpath}/{final_romfs_file}").is_file(): #romfs file
            os.remove(f"{newpath}/{final_romfs_file}")
    if Path(f"{newpath}/errors.txt").is_file(): #errors file
            os.remove(f"{newpath}/errors.txt")
    
    #create the folder and check files
    if not os.path.exists(vehicle_name):
        os.makedirs(vehicle_name)
    if not params_path.is_file():
        raise ValueError("Error: params_path must be a valid file path.")
    if not romfs_path.is_file():
        raise ValueError("Error: romfs_path must be a valid file path.")
    
    with open(romfs_path, 'r') as tf, open(final_romfs_file, 'w') as ff:
        shutil.copyfileobj(tf, ff)

    #move files to the folder
    shutil.move(final_romfs_file, f"{newpath}/")
    shutil.move(params_file, f"{newpath}/")
    #convert from the QGC designation to the gazebo designation
    process_params_file(f"{newpath}/{params_file}", f"{newpath}/{final_romfs_file}")

    #place the new ROMFS file on the ROMFS directory and run gazebo to find missing parameters
    final_romfs_path = airframes_path + final_romfs_file
    shutil.copy(f"{newpath}/{final_romfs_file}", final_romfs_path)

    print(f"\033[92m[INFO]\033[0m: passed the parameters from QGC to gazebo{final_romfs_file}")

    return [final_romfs_file, final_romfs_path]

if __name__ == '__main__':
    main()
