#!/usr/bin/env
#The idea of this file is to get some user inputs and construct an appropriate AVL file from this.


#Add in support for custom run case naming and case for when a particular file already exists.

#Prepare liftdrag template here as you will need to set the correct number of control surfaces.

import webbrowser
import avl_out_parse
import os
import yaml
import subprocess



# This function writes section definition to AVL file
def write_section(plane_name,x,y,z,chord,ainc,nspan,sspace,naca_number,ctrl_surf_type):

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

# This is a helper function to securely split a vector into three elements and retry if it fails
def split_into_three(vector,message):
    while(len(vector)!= 3):
        print("This is not the right number of elements. Try again")
        vector = input(message).split()
        try:
            vector[0] = float(vector[0])
            vector[1] = float(vector[1])
            vector[2] = float(vector[2])
        except:
            print("At least of the coordinates is not a number!")
            vector = []


    return float(vector[0]), float(vector[1]), float(vector[2])


def main():

    with open('input.yml','r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)

    not_valid_a = True
    airframes = ['cessna','standard_vtol','custom']
    # plane_name = input("Please enter a name for your vehicle: ")
    plane_name = yaml_data['vehicle_name']
    print("Choose from predetermined models or define a custom model. Current options for airframes are:",airframes,'\n')
    print("For a custom model, write 'custom'. \n")


    while(not_valid_a):
        # frame_type = input("Please enter the type of airframe you would like to use: ")
        frame_type = yaml_data['frame_type']
        if not frame_type in airframes:
            print("\nThis is not a valid airframe, please select a valid airframe. \n")
        else:
            not_valid_a = False

    # Set model specific parameters
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

    # TODO: Provide some pre-worked frames for a Cessna and standard VTOL
    match frame_type:
        case "cessna":
            num_ctrl_surfaces = 4
            # TODO: Finish

        case "standard_vtol":
            num_ctrl_surfaces = 2
            # TODO: Finish

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

            print("First define some model-specific parameters for custom models: ")
            area = yaml_data["reference_area"]
            span = yaml_data["wing_span"]
            ref_pt_x = yaml_data["reference_point"]["X"]
            ref_pt_y = yaml_data["reference_point"]["Y"]
            ref_pt_z = yaml_data["reference_point"]["Z"]

            if(span != 0 and area != 0):
                ref_chord = float(area)/float(span)
            else:
                raise ValueError("Invalid reference chord. Check area and wing span")

            # Write the gather model-wide parameters into the .avl file
            with open(f'{plane_name}.avl','a') as avl_file:
                avl_file.write('!Sref    Cref    Bref \n')
                avl_file.write(f'{area}     {str(ref_chord)}     {span} \n')
                avl_file.write('!Xref    Yref    Zref \n')
                avl_file.write(f'{ref_pt_x}     {ref_pt_y}      {ref_pt_z} \n')
                avl_file.close()


            num_ctrl_surfaces = yaml_data["num_ctrl_surfaces"]

            print("\nDefine parameter for EACH control surface \n")

            for i, control_surface in enumerate(yaml_data["control_surfaces"]):

                # Wings always need to be defined from left to right
                ctrl_surf_name = control_surface['name']
                print("Valid control surface types are: ",ctrl_surface_types)
                not_valid_ctrl_surface = True

                while(not_valid_ctrl_surface):
                    ctrl_surf_type = control_surface['type']
                    if ctrl_surf_type not in ctrl_surface_types:
                        print("Not a valid type of control surface! \n")
                    else:
                        not_valid_ctrl_surface = False

                # The order of control surfaces becomes important in the output parsing
                # to correctly assign derivatives to particular surfaces.
                ctrl_surface_order.append(ctrl_surf_type)

                nchord = control_surface["nchord"]
                cspace = control_surface["cspace"]
                nspanwise = control_surface["nspan"]
                sspace = control_surface["sspace"]

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

                    # If we have a rudder, we can duplicate the defined control surface along the y-axis of the model
                    # as both sides are generally modelled and controlled as one in simulation.
                    if ctrl_surf_type.lower() == 'elevator':
                        avl_file.write("\nYDUPLICATE\n")
                        avl_file.write("0.0\n\n")

                    if ctrl_surf_type.lower() == 'aileron':
                        avl_file.write("ANGLE \n")
                        avl_file.write(f'{angle} \n')

                    avl_file.write("TRANSLATE \n")
                    avl_file.write(f'{tx}    {ty}    {tz} \n')
                    avl_file.close()


                # Define NACA airfoil shape. Only used for aileron.
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


    AR = str((float(span)*float(span))/float(area))
    mac = str((2/3)*(float(area)/float(span)))

    # Call shell script that will pass the generated .avl file to AVL
    os.system(f'./process.sh {plane_name}')

    # Call main function of avl parse script
    avl_out_parse.main(plane_name,frame_type,AR,mac,ref_pt_x,ref_pt_y,ref_pt_z,num_ctrl_surfaces,area,ctrl_surface_order)

    # Finally move all generated files to a new directory and show the generated geometry image:
    result = subprocess.run(['pwd'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if result.returncode == 0:
        # Save the output in a variable
        current_path = result.stdout.strip()

    os.system(f'mv ./{plane_name}.* ./{plane_name}' )
    os.system(f'evince {current_path}/{plane_name}/{plane_name}.ps')

if __name__ == '__main__':
    main()
