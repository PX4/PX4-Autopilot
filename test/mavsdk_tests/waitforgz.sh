#!/usr/bin/env bash

while gz stats -d 0 2>&1 | grep "An instance of Gazebo is not running."; do
    echo "Gazebo not running yet ..."
done
