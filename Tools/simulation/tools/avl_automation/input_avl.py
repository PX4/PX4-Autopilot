#!/usr/bin/env

import argparse
import avl_out_parse
import os
import yaml
import subprocess
import shutil

"""
Write individual airfoil section definitions to the .avl file.
Sections are defined through a 3D point in space and assigned properties such as chord, angle of incidence etc.
AVL then links them up to the other sections of a particular surface. You can define any number of sections for
a particular surface, but there always have to be at least two (a left and right edge).

Args:
	plane_name (str): The name of the vehicle.
	x (str): The x coordinate of the section.
	y (str): The y coordinate of the section.
	z (str): The z coordinate of the section.
	chord (str): Chord in this section of the surface. Trailing edge is at x + chord, y, z.
	ainc (str): Angle of incidence for this section. Taken as a rotation (RH rule) about the surface's
		spanwise axis projected onto the Y-Z plane.
	nspan (str): Number of spanwise vortices in until the next section.
	sspan (str): Controls the spanwise spacing of the vortices.
	naca_number (str): The chosen NACA number that will define the cambered properties of this section
		of the surface. For help picking an airfoil go to: http://airfoiltools.com/airfoil/naca4digit.
	ctrl_surface_type: The selected type of control surface. This should be consistent along the entirety of
		the surface. (Question: Flap and Aileron along the same airfoil?)

Return:
	None.

"""
def write_section(plane_name: str,x: str,y: str,z: str,chord: str,ainc: str,nspan: str,sspace: str,naca_number: str,ctrl_surf_type: str):

	with open(f'{plane_name}.avl','a') as avl_file:
		avl_file.write("SECTION \n")
		avl_file.write("!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n")
		avl_file.write(f'{x}   {y}    {z}    {chord}    {ainc}     {nspan}    {sspace} \n')
		if naca_number != "0000":
			avl_file.write("NACA \n")
			avl_file.write(f'{naca_number} \n')
		avl_file.close()

	match ctrl_surf_type:
		case 'aileron':
			#TODO provide custom options for gain and hinge positions
			with open(f'{plane_name}.avl','a') as avl_file:
				avl_file.write("CONTROL \n")
				avl_file.write("aileron  1.0  0.0  0.0  0.0  0.0  -1 \n")
				avl_file.close()

		case 'elevator':
			with open(f'{plane_name}.avl','a') as avl_file:
				avl_file.write("CONTROL \n")
				avl_file.write("elevator  1.0  0.0  0.0  0.0  0.0  1 \n")
				avl_file.close()

		case 'rudder':
			with open(f'{plane_name}.avl','a') as avl_file:
				avl_file.write("CONTROL \n")
				avl_file.write("rudder  1.0  0.0  0.0  0.0  0.0  1 \n")
				avl_file.close()



"""
Read the provided yaml file and generate the corresponding .avl file that can be read into AVL.
Also calls AVL and the avl_out_parse.py file that generates the sdf plugin.

Args:
	yaml_file: Path to the input yaml file
	avl_path: Set the avl_path to provide a desired directory for where Avl should be located.

Return:
	None

"""
def main():
	user = os.environ.get('USER')
	# This will find Avl on a users machine.
	for root, dirs, _ in os.walk(f'/home/{user}/'):
		if "Avl" in dirs:
			target_directory_path = os.path.join(root, "Avl")
			break
	parent_directory_path = os.path.dirname(target_directory_path)
	filedir = f'{parent_directory_path}/'
	print(filedir)

	parser = argparse.ArgumentParser()
	parser.add_argument("--yaml_file", help="Path to input yaml file.")
	parser.add_argument("--avl_path", default=filedir, help="Provide an absolute AVL path. If this argument is passed, AVL will be moved there and the files will adjust their paths accordingly.")
	inputs = parser.parse_args()


	# If the user passes the avl_path argument then move Avl to that location:
	if inputs.avl_path != filedir:

		#Check if the directory is already there
		if os.path.exists(f'{inputs.avl_path}/Avl') and os.path.isdir(f'{inputs.avl_path}/Avl'):
			print("Avl is already at desired location")
		else:
			shutil.move(f'{filedir}Avl',inputs.avl_path)

		# Adjust paths to AVL in process.sh
		print("Adjusting paths")
		with open("./process.sh", "r") as file:
			all_lines = file.readlines()
			file.close()

		it = 0
		for line in all_lines:
			if "cp $DIR_PATH/$CUSTOM_MODEL.avl" in line:
				new_line = f'cp $DIR_PATH/$CUSTOM_MODEL.avl {inputs.avl_path}Avl/runs\n'
				all_lines[it] = new_line

			if "/Avl/runs/plot.ps $DIR_PATH/" in line:
				new_line =f'mv {inputs.avl_path}Avl/runs/plot.ps $DIR_PATH/\n'
				all_lines[it] = new_line

			if "cd" in line and "/Avl/runs" in line:
				new_line = f'cd {inputs.avl_path}Avl/runs\n'
				all_lines[it] = new_line
			it += 1

		with open("./process.sh", "w") as file:
			file.writelines(all_lines)
			file.close()


	with open(inputs.yaml_file,'r') as yaml_file:
		yaml_data = yaml.safe_load(yaml_file)

	airframes = ['cessna','standard_vtol','custom']
	plane_name = yaml_data['vehicle_name']
	frame_type = yaml_data['frame_type']
	if not frame_type in airframes:
		raise ValueError("\nThis is not a valid airframe, please choose a valid airframe. \n")

	# Parameters that need to be provided:
	# General
	# - Reference Area (Sref)
	# - Wing span (Bref) (wing span squared / area = aspect ratio which is a required parameter for the sdf file)
	# - Reference point (X,Y,Zref) point at which moments and forces are calculated
	#Control Surface specific
	# - type (select from options; aileron,elevator,rudder)
	# - nchord
	# - cspace
	# - nspanwise
	# - sspace
	# - x,y,z 1. (section)
	# - chord 1. (section)
	# - ainc 1. (section)
	# - Nspan 1. (optional for section)
	# - sspace 1. (optional for section)
	# - x,y,z 2. (section)
	# - chord 2. (section)
	# - ainc 2. (section)
	# - Nspan 2. (optional for section)
	# - sspace 2. (optional for section)

	# TODO: Find out if elevons are defined
	ctrl_surface_types = ['aileron','elevator','rudder']
	# - Reference Chord (Cref) (= area/wing span)
	delineation = '!***************************************'
	sec_demark = '#--------------------------------------------------'
	num_ctrl_surfaces = 0
	ctrl_surface_order = []
	area = 0
	span = 0

	ref_pt_x = None
	ref_pt_y = None
	ref_pt_z = None

	# Future work: Provide some pre-worked frames for a Cessna and standard VTOL if there is a need for it
	match frame_type:

		case "custom":

			# These parameters are consistent across all models.
			# At the moment we do not use any symmetry axis for mirroring.
			with open(f'{plane_name}.avl','w') as avl_file:
				avl_file.write(f'{delineation} \n')
				avl_file.write(f'!{plane_name} input dataset \n')
				avl_file.write(f'{delineation} \n')
				avl_file.write(f'{plane_name} \n')
				avl_file.write('!Mach \n0.0 \n')
				avl_file.write('!IYsym    IZsym    Zsym \n')
				avl_file.write('0     0     0 \n')
				avl_file.close()

			# First define some model-specific parameters for custom models
			area = yaml_data["reference_area"]
			span = yaml_data["wing_span"]
			ref_pt_x = yaml_data["reference_point"]["X"]
			ref_pt_y = yaml_data["reference_point"]["Y"]
			ref_pt_z = yaml_data["reference_point"]["Z"]

			if(span != 0 and area != 0):
				ref_chord = float(area)/float(span)
			else:
				raise ValueError("Invalid reference chord value. Check area and wing span values.")

			# Write the gathered model-wide parameters into the .avl file
			with open(f'{plane_name}.avl','a') as avl_file:
				avl_file.write('!Sref    Cref    Bref \n')
				avl_file.write(f'{area}     {str(ref_chord)}     {span} \n')
				avl_file.write('!Xref    Yref    Zref \n')
				avl_file.write(f'{ref_pt_x}     {ref_pt_y}      {ref_pt_z} \n')
				avl_file.close()

			num_ctrl_surfaces = yaml_data["num_ctrl_surfaces"]
			for i, control_surface in enumerate(yaml_data["control_surfaces"]):

				# Wings always need to be defined from left to right
				ctrl_surf_name = control_surface['name']
				ctrl_surf_type = control_surface['type']
				if ctrl_surf_type not in ctrl_surface_types:
					raise ValueError(f'The selected type is invalid. Available types are: {ctrl_surface_types}')

				# The order of control surfaces becomes important in the output parsing
				# to correctly assign derivatives to particular surfaces.
				ctrl_surface_order.append(ctrl_surf_type)

				nchord = control_surface["nchord"]
				cspace = control_surface["cspace"]
				nspanwise = control_surface["nspan"]
				sspace = control_surface["sspace"]

				# TODO: Add more control surface types that also require Angles.
				if ctrl_surf_type.lower() == 'aileron':
					angle = control_surface["angle"]

				#Translation of control surface, will move the whole surface to specified position
				tx = control_surface["translation"]["X"]
				ty = control_surface["translation"]["Y"]
				tz = control_surface["translation"]["Z"]

				# Write common part of this surface to .avl file
				with open(f'{plane_name}.avl','a') as avl_file:
					avl_file.write(sec_demark)
					avl_file.write("\nSURFACE \n")
					avl_file.write(f'{ctrl_surf_name} \n')
					avl_file.write("!Nchordwise     Cspace      Nspanwise       Sspace \n")
					avl_file.write(f'{nchord}       {cspace}        {nspanwise}     {sspace} \n')

					# If we have a elevator, we can duplicate the defined control surface along the y-axis of the model
					# as both sides are generally modelled and controlled as one in simulation. Adjust for split elevators if desired.
					if ctrl_surf_type.lower() == 'elevator':
						avl_file.write("\nYDUPLICATE\n")
						avl_file.write("0.0\n\n")

					# Elevators and Rudders do not require an angle of incidence.
					if ctrl_surf_type.lower() == 'aileron':
						avl_file.write("ANGLE \n")
						avl_file.write(f'{angle} \n')

					# Translate the surface to a particular position in space.
					avl_file.write("TRANSLATE \n")
					avl_file.write(f'{tx}    {ty}    {tz} \n')
					avl_file.close()


				# Define NACA airfoil shape.
				# For help picking an airfoil go to: http://airfoiltools.com/airfoil/naca4digit
				# NOTE: AVL can only use 4-digit NACA codes.
				if ctrl_surf_type.lower() == "aileron":
					naca_number = control_surface["naca"]
				else:
					# Provide a default NACA number for unused airfoils
					naca_number = '0000'

				# Iterating over each defined section for the control surface. There need to be at least
				# two in order to define a left and right edge, but there is no upper limit.
				# CRITICAL: ALWAYS DEFINE YOUR SECTION FROM LEFT TO RIGHT
				for j, section in enumerate(control_surface["sections"]):

					print(f'Defining {j}. section of {i+1}. control surface \n')
					y = section["position"]["Y"]
					z = section["position"]["Z"]
					x = section["position"]["X"]
					chord = section["chord"]
					ainc = section["ainc"]
					nspan = section["nspan"]
					write_section(plane_name,x,y,z,chord,ainc,nspan,sspace,naca_number,ctrl_surf_type)

				print(f'\nPARAMETER DEFINITION FOR {i+1}. CONTROL SURFACE COMPLETED \n')


   	# Calculation of Aspect Ratio (AR) and Mean Aerodynamic Chord (mac)
	AR = str((float(span)*float(span))/float(area))
	mac = str((2/3)*(float(area)/float(span)))

	# Call shell script that will pass the generated .avl file to AVL
	os.system(f'./process.sh {plane_name}')

	# Call main function of avl parse script to parse the generated AVL files.
	avl_out_parse.main(plane_name,frame_type,AR,mac,ref_pt_x,ref_pt_y,ref_pt_z,num_ctrl_surfaces,area,ctrl_surface_order,inputs.avl_path)

	# Finally move all generated files to a new directory and show the generated geometry image:
	result = subprocess.run(['pwd'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

	if result.returncode == 0:
		# Save the output in a variable
		current_path = result.stdout.strip()

	# Run image plot from avl_automation directory.
	os.system(f'mv ./{plane_name}.* ./{plane_name}' )
	os.system(f'evince {current_path}/{plane_name}/{plane_name}.ps')

if __name__ == '__main__':
	main()
