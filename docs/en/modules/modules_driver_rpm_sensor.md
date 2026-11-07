# Modules Reference: Rpm Sensor (Driver)

::: info
**This page is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4moduledoc/markdownout.py) from the source code** using the build command: `make module_documentation`.
Do not edit it directly; fix the module documentation in the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) source instead.
:::

## pcf8583

Source: [drivers/rpm/pcf8583](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rpm/pcf8583)

### Usage {#pcf8583_usage}

```
pcf8583 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es) (onboard sensors; shared bus only if -b is
                 given)
     [-X]        External I2C bus(es) (connector sensors, including shared
                 buses)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 80
     [-k]        if initialization (probing) fails, keep retrying periodically

   stop

   status        print status info
```
