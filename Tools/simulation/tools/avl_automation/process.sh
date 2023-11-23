#!/bin/bash
CUSTOM_MODEL=$1
DIR_PATH=$(pwd)

cp $DIR_PATH/$CUSTOM_MODEL.avl /home/$USER/Avl/runs/
cd
cd /home/$USER/Avl/runs

old_stability_derivatives="custom_vehicle_stability_derivatives.txt"
old_body_ax_derivatives="custom_vehicle_body_axis_derivatives.txt"

if [ -e "$old_stability_derivatives" ]; then
    # Delete old stability derivative file
    rm "$old_stability_derivatives"
fi
if [ -e "$old_body_ax_derivatives" ]; then
    # Delete old body_axis derivative file
    rm "$old_body_ax_derivatives"
fi

#avl_steps.txt can be used to run commands on the AVL commandline.
../bin/avl $CUSTOM_MODEL.avl < $DIR_PATH/avl_steps.txt
echo "\n"

#After completion move the plot to avl_automation directory
mv /home/$USER/Avl/runs/plot.ps $DIR_PATH/
mv $DIR_PATH/plot.ps $DIR_PATH/$CUSTOM_MODEL.ps
