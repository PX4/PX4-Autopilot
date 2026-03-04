# Modules Reference: Template

## mc_raptor

Source: [modules/mc_raptor](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_raptor)

### Description

RAPTOR Policy Flight Mode

### Usage {#mc_raptor_usage}

```
mc_raptor <command> [arguments...]
 Commands:
   start

   intref        Modify internal reference
     lissajous   Set Lissajous trajectory parameters
     <A>         Amplitude X [m]
     <B>         Amplitude Y [m]
     <C>         Amplitude Z [m]
     <fa>        Frequency a
     <fb>        Frequency b
     <fc>        Frequency c
     <duration>  Total duration [s]
     <ramp>      Ramp duration [s]

   stop

   status        print status info
```

## module

Source: [templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module)

### Description

Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation

Section describing the high-level implementation of this module.

### Examples

CLI usage example:

```
module start -f -p 42
```

### Usage {#module_usage}

```
module <command> [arguments...]
 Commands:
   start
     [-f]        Optional example flag
     [-p <val>]  Optional example parameter
                 default: 0

   stop

   status        print status info
```

## work_item_example

Source: [examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/work_item)

### Description

Example of a simple module running out of a work queue.

### Usage {#work_item_example_usage}

```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
