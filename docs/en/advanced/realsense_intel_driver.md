# Installing driver on Ubuntu for Intel RealSense R200

This tutorial aims to give instructions on how to install the camera driver of the Intel RealSense R200 camera head in Linux environment such that the gathered images can be accessed via the Robot Operation System (ROS).
The RealSense R200 camera head is depicted below:

![Intel Realsense Camera front view](../../assets/hardware/sensors/realsense/intel_realsense.png)

The installation of the driver package is executed on a Ubuntu operation system (OS) that runs as a guest OS in a Virtual Box.
The specifications of the host computer where the Virtual Box is running, the Virtual Box and the guest system are given below:

- Host Operation System: Windows 8
- Processor: Intel(R) Core(TM) i7-4702MQ CPU @ 2.20GHz
- Virtual Box: Oracle VM. Version 5.0.14 r105127
- Extensions: Extension package for Virtual Box installed (Needed for USB3 support)
- Guest Operation System: Linux - Ubuntu 14.04.3 LTS

The tutorial is ordered in the following way: In a first part it is shown how to install Ubuntu 14.04 as a guest OS in the Virtual Box. In a second part is shown how to install ROS Indigo and the camera driver. The ensuing frequently used expressions have the following meaning:

- Virtual Box (VB): Program that runs different Virtual Machines. In this case the Oracle VM.
- Virtual Machine (VM): The operation system that runs in the Virtual Box as a guest system. In this case Ubuntu.

## Installing Ubuntu 14.04.3 LTS in Virtual Box

- Create a new Virtual Machine (VM): Linux 64-Bit.
- Download the iso file of Ubuntu 14.04.3 LTS: ([ubuntu-14.04.3-desktop-amd64.iso](https://ubuntu.com/download/desktop)).
- Installation of Ubuntu:
  - During the installation procedure leave the following two options unchecked:
    - Download updates while installing
    - Install this third party software
- After the installation you might need to enable the Virtual Box to display Ubuntu on the whole desktop:
  - Start VM Ubuntu and login, Click on **Devices->Insert Guest Additions CD image** in the menu bar of the Virtual Box.
  - Click on **Run** and enter password on the windows that pop up in Ubuntu.
  - Wait until the installation is completed and then restart.
    Now, it should be possible to display the VM on the whole desktop.
  - If a window pops up in Ubuntu that asks whether to update, reject to update at this point.
- Enable USB 3 Controller in Virtual Box:
  - Shut down Virtual Machine.
  - Go to the settings of the Virtual Machine to the menu selection USB and choose: "USB 3.0(xHCI)".
    This is only possible if you have installed the extension package for the Virtual Box.
  - Start the Virtual Machine again.

## Installing ROS Indigo

- Follow instructions given at [ROS indigo installation guide](http://wiki.ros.org/indigo/Installation/Ubuntu):
  - Install Desktop-Full version.
  - Execute steps described in the sections "Initialize rosdep" and "Environment setup".

## Installing camera driver

- Install git:

  ```sh
  sudo apt-get install git
  ```

- Download and install the driver:

  - Clone [RealSense_ROS repository](https://github.com/bestmodule/RealSense_ROS):

    ```sh
    git clone https://github.com/bestmodule/RealSense_ROS.git
    ```

- Follow instructions given in [here](https://github.com/bestmodule/RealSense_ROS/tree/master/r200_install).

  - Press the enter button when the questions whether to install the following installation packages show up:

    ```sh
    Intel Low Power Subsystem support in ACPI mode (MFD_INTEL_LPSS_ACPI) [N/m/y/?] (NEW)
    ```

    ```sh
    Intel Low Power Subsystem support in PCI mode (MFD_INTEL_LPSS_PCI) [N/m/y/?] (NEW)
    ```

    ```sh
    Dell Airplane Mode Switch driver (DELL_RBTN) [N/m/y/?] (NEW)
    ```

  - The following error message that can appear at the end of the installation process should not lead to a malfunction of the driver:

    ```sh
    rmmod: ERROR: Module uvcvideo is not currently loaded
    ```

- After the installation has completed, reboot the Virtual Machine.

- Test camera driver:

  - Connect the Intel RealSense camera head with the computer with a USB3 cable that is plugged into a USB3 receptacle on the computer.
  - Click on Devices->USB-> Intel Corp Intel RealSense 3D Camera R200 in the menu bar of the Virtual Box, in order to forward the camera USB connection to the Virtual Machine.
  - Execute the file [unpacked folder]/Bin/DSReadCameraInfo:

    - If the following error message appears, unplug the camera (physically unplug USB cable from the computer). Plug it in again + Click on Devices->USB-> Intel Corp Intel RealSense 3D Camera R200 in the menu bar of the Virtual Box again and execute again the file [unpacked folder]/Bin/DSReadCameraInfo.

      ```sh
      DSAPI call failed at ReadCameraInfo.cpp:134!
      ```

    - If the camera driver works and recognises the Intel RealSense R200, you should see specific information about the Intel RealSense R200 camera head.

- Installation and testing of the ROS nodlet:
  - Follow the installation instructions in the "Installation" section given [here](https://github.com/bestmodule/RealSense_ROS/blob/master/realsense_dist/2.3/doc/RealSense-ROS-R200-nodelet.md), to install the ROS nodlet.
  - Follow the instructions in the "Running the R200 nodelet" section given [here](https://github.com/bestmodule/RealSense_ROS/blob/master/realsense_dist/2.3/doc/RealSense-ROS-R200-nodelet.md), to test the ROS nodlet together with the Intel RealSense R200 camera head.
    - If everything works, the different data streams from the Intel RealSense R200 camera are published as ROS topics.
