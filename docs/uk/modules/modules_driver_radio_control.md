# Modules Reference: Radio Control (Driver)

## crsf_rc

Source: [drivers/rc/crsf_rc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rc/crsf_rc)

### Опис

This module parses the CRSF RC uplink protocol and generates CRSF downlink telemetry data

### Usage {#crsf_rc_usage}

```
crsf_rc <command> [arguments...]
 Commands:
   start
     [-d <val>]  RC device
                 values: <file:dev>, default: /dev/ttyS3

   stop

   status        print status info
```

## dsm_rc

Source: [drivers/rc/dsm_rc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rc/dsm_rc)

### Опис

This module does Spektrum DSM RC input parsing.

### Usage {#dsm_rc_usage}

```
dsm_rc <command> [arguments...]
 Commands:
   start
     [-d <val>]  RC device
                 values: <file:dev>, default: /dev/ttyS3

   bind          Send a DSM bind command (module must be running)

   stop

   status        print status info
```

## ghst_rc

Source: [drivers/rc/ghst_rc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rc/ghst_rc)

### Опис

This module does Ghost (GHST) RC input parsing.

### Usage {#ghst_rc_usage}

```
ghst_rc <command> [arguments...]
 Commands:
   start
     [-d <val>]  RC device
                 values: <file:dev>, default: /dev/ttyS3

   stop

   status        print status info
```

## rc_input

Source: [drivers/rc_input](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rc_input)

### Опис

This module does the RC input parsing and auto-selecting the method. Supported methods are:

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

### Usage {#rc_input_usage}

```
rc_input <command> [arguments...]
 Commands:
   start
     [-d <val>]  RC device
                 values: <file:dev>, default: /dev/ttyS3

   bind          Send a DSM bind command (module must be running)

   stop

   status        print status info
```

## sbus_rc

Source: [drivers/rc/sbus_rc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/rc/sbus_rc)

### Опис

This module does SBUS RC input parsing.

### Usage {#sbus_rc_usage}

```
sbus_rc <command> [arguments...]
 Commands:
   start
     [-d <val>]  RC device
                 values: <file:dev>, default: /dev/ttyS3

   stop

   status        print status info
```
