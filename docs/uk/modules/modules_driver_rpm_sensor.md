# Modules Reference: Rpm Sensor (Driver)

## pcf8583

Source: [drivers/rpm/pcf8583](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rpm/pcf8583)

<a id="pcf8583_usage"></a>

### Використання

```
pcf8583 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
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
