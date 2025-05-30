# Advanced Linux Installation Use-Cases

## Using JTAG Programming Adapters

Linux users need to explicitly allow access to the USB bus for JTAG programming adapters.

::: info
For Archlinux: replace the group plugdev with uucp in the following commands
:::

Run a simple `ls` in `sudo` mode to ensure the commands below succeed:

```sh
sudo ls
```

Then with `sudo` rights temporarily granted, run this command:

```sh
cat > $HOME/rule.tmp <<_EOF
# All 3D Robotics (includes PX4) devices
SUBSYSTEM=="usb", ATTR{idVendor}=="26AC", GROUP="plugdev"
# FTDI (and Black Magic Probe) Devices
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", GROUP="plugdev"
# Olimex Devices
SUBSYSTEM=="usb",  ATTR{idVendor}=="15ba", GROUP="plugdev"
_EOF
sudo mv $HOME/rule.tmp /etc/udev/rules.d/10-px4.rules
sudo /etc/init.d/udev restart
```

The user needs to be added to the group **plugdev**:

```sh
sudo usermod -a -G plugdev $USER
```
