#!/bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $script_dir
if [[ $1 == "-h" ]]; then
    echo "Usage: maneuver [maneuver-name]"
    exit 1
fi

build_dir=$script_dir/Maneuvers/build

# check if build folder exists
if [[ ! -d $build_dir ]]; then
    echo "No build folder present"
    echo "Do you want build Maneuvers?"
    read -p "Continue (y/n)?" answer

    if [[ $answer == [Yy] ]]; then
        mkdir $build_dir
        cd $build_dir
        cmake ../src &&  make
        cd $SCRIPT_DIR
    else
        exit
    fi
fi

files=$build_dir/maneuvers/*
input=$build_dir/maneuvers/$1
# check if requeste maneuver exists
for f in $files
do
    if [[ -x $f && ! -d $f && $f == $input ]]; then
        maneuver=$1
    fi
done

if [[ -z $maneuver ]]; then
    echo "This maneuver does not exist."
    echo "The available maneuvers are:"
    for f in $files
    do
        if [[ -x $f && ! -d $f ]]; then
            echo $(basename $f)
        fi
    done
    exit
fi

# start requested maneuver
cd $build_dir && ./maneuvers/$maneuver udp://:14540