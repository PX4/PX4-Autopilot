## Atmel Same70 Xplained ##

This is the board config that runs on the Atmel Same70 Xplained development board.

http://www.atmel.com/tools/ATSAME70-XPLD.aspx

1. git clone https://github.com/PX4/Firmware.git
2. git checkout same70xplained
3. make clean
4. git submudule update --init --recursive
5. make px4-same70xplained-v1_default

The ELF file Firmare will be in `build_px4-same70xplained-v1_default/src/firmware/nuttx/`

The EFL file name is `firmware_nuttx`

Pin out Same70 Xplained development board is:
![Same70 Xplained development board](https://cloud.githubusercontent.com/assets/1945821/15483794/615ff9ec-20d2-11e6-918b-628dc52374fe.png "Pinout on Same70 Xplained development board")
May-3-2016 Changes are highlited in orange
May-23-2016 Changes are highlited in Blue

![Drotek UBLOX NEO-M8N GPS + HMC5983 COMPASS (XL) - wiring](https://cloud.githubusercontent.com/assets/1945821/15004599/b249859a-1154-11e6-8e54-4af891f9cf85.png)

![DroTek MPU9250 - wiring](https://cloud.githubusercontent.com/assets/1945821/15484096/f5b8336a-20d3-11e6-80f3-2b4f9dc3f120.png)
