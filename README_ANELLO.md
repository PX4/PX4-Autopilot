# ANELLO Maritime INS Firmware Setup

After flashing the ARKV6X or Cube Orange Plus ANELLO firmware, verify these parameters in QGroundControl.
They are firmware defaults, but **previously saved values can override them**.
Use the firmware image built for your specific board.

## Parameter Settings

| Parameter | Required value | Purpose |
| --- | --- | --- |
| `MAV_0_CONFIG` | `0` — Disabled | Frees TELEM1 from the default MAVLink instance |
| `SENS_ANELLO_CFG` | `101` — TELEM1 | Assigns TELEM1 to the ANELLO driver |
| `ANELLO_BAUD` | `921600` | Sets the INS serial baud rate |
| `ANELLO_MODE` | `1` — Primary | Uses ANELLO as the primary navigation estimator |
| `ANELLO_AUTOCFG` | `1` — Enabled | Requests 50 Hz attitude and 10 Hz position from the INS |
| `EKF2_EN` | `0` — Disabled | Disables PX4's onboard EKF2 estimator |
| `ATT_EN` | `0` — Disabled | Disables the alternative attitude estimator, if this parameter exists |
| `SENS_IMU_MODE` | `1` | Keeps onboard IMU selection available for rate control |
| `ANELLO_EPH` | `100` m | Assumed horizontal position uncertainty |
| `ANELLO_EPV` | `100` m | Assumed vertical position uncertainty |
| `ANELLO_EV` | `100` m/s | Assumed velocity uncertainty |
| `SYS_HAS_GPS` | `0` | Specifies that no separate GPS receiver is installed |

**The three uncertainty values are requested test settings, not measured INS accuracy. They may prevent PX4 position-control checks from passing.**
These settings assume ANELLO is the only navigation source and no separate GPS receiver is installed.

## Connection And Restart

Connect the INS to TELEM1 with TX, RX, and ground, using compatible electrical levels.
Ensure no other serial driver is assigned to TELEM1, and reboot after setting the parameters.
TELEM1 maps to `/dev/ttyS6` on ARKV6X and `/dev/ttyS0` on Cube Orange Plus.
`SER_TEL1_BAUD` does not need changing: the ANELLO driver uses `ANELLO_BAUD`.
Preserve the board's sensor calibrations; a full parameter reset is unnecessary.

## Verify Reception

After reboot, run this in QGroundControl's MAVLink Console:

```sh
anello status
```

Check for increasing received bytes and accepted attitude/position counts, primary mode, and stream setup showing `accepted: 3/3`.
An accepted rate request confirms the command response, not navigation quality or the measured output rate.

For driver behaviour, limitations, and further checks, see [ANELLO Maritime INS documentation](docs/en/sensor/anello_maritime.md).
