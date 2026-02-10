# PX4 Autopilot Firmware for FSR@TUDa

Base Version: **v1.15.4**

This fork adds airframes and board configurations required for operating the UAS fleet of FSR@TUDa.

## Additional Board Configuration

### `boards/px4/fmu-v6x`

- [crsf.px4board](boards/px4/fmu-v6x/crsf.px4board)

  Enable crossfire as RC.

## Additional Airframes

### `ROMFS/px4fmu_common/init.d/airframes`

- [4002_tuda_fsr_edufly](ROMFS/px4fmu_common/init.d/airframes/4002_tuda_fsr_edufly)
- [13001_tuda_fsr_scidragon](ROMFS/px4fmu_common/init.d/airframes/13001_tuda_fsr_scidragon)

### `ROMFS/px4fmu_common/init.d-posix/airframes` (Simulation)
