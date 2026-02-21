# libmodal-json

libmodal-json is an extension of Dave Gamble's excellent cJSON library with the following helper functions to facilitate the most common JSON file operation used at ModalAI for config and calibration files. All the cJSON functions and types are included in this header and library to give the user complete low-level control.
If there isn't a helper function in the header for something you want to do, look through cJSON.h and you will find what you need!

For a tutorial on how to use this library, read through the modal_test_json.c example program.


## Build dependencies

none!

## Build Environment

This project builds in the voxl-cross docker image

Follow the instructions here to build and install the voxl-cross docker image:
https://gitlab.com/voxl-public/voxl-docker


## Build Instructions

1) Launch the voxl-cross docker.

```bash
~/git/libmodal-json$ voxl-docker -i voxl-cross
voxl-cross(4.3):~$
```

2) Install dependencies inside the docker. You must specify both the hardware platform and binary repo section to pull from. CI will use the `dev` binary repo for `dev` branch jobs, otherwise it will select the correct target SDK-release based on tags. When building yourself, you must decide what your intended target is, usually `dev`

You must also specify if you are building for a 1.X VOXL 2 system image (Ubtuntu 18.04) or a 2.X VOXL2 system image (Ubuntu 20.04). These are specified as "qrb5165" and "qrb5165-2" platforms respectively.



```bash
voxl-cross(4.3):~$ ./install_build_deps.sh qrb5165 dev
OR
voxl-cross(4.3):~$ ./install_build_deps.sh qrb5165-2 dev
```


3) Build scripts should take the hardware platform as an argument: `qrb5165` or `qrb5165-2`. CI will pass these arguments to the build script based on the job target. 


```bash
voxl-cross(4.3):~$ ./build.sh qrb5165
OR
voxl-cross(4.3):~$ ./build.sh qrb5165-2
```


4) Make a deb package while still inside the docker.

```bash
voxl-cross(4.3):~$ ./make_package.sh
```

This will make a new .deb package file in your working directory. The name and version number came from the package control file. If you are updating the package version, edit it there.

Optionally add the --timestamp argument to append the current data and time to the package version number in the debian package. This is done automatically by the CI package builder for development and nightly builds, however you can use it yourself if you like.


## Run Linter Check (optional)

```bash
voxl-cross(4.3):~$ voxl-cross-linter
```

## Deploy to VOXL

You can now push the deb package to VOXL and install with dpkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

```bash
(outside of docker)
libmodal-json$ ./deploy_to_voxl.sh
```

This deploy script can also push over a network given sshpass is installed and the VOXL uses the default root password.


```bash
(outside of docker)
libmodal-json$ ./deploy_to_voxl.sh ssh 192.168.1.xyz
```

