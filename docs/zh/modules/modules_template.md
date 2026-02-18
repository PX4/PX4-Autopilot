# 模块参考: 模板

## mc_raptor

Source: [modules/mc_raptor](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_raptor)

### 描述

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

## 模块

Source: [templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module)

### 描述

该部分描述所提供模块的功能。

这是一个模块的模版，该模块在后台作为任务（task）运行并且有 start/stop/status 功能。

### 实现

该部分描述模块的高层次实现。

### 示例

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

### 描述

Example of a simple module running out of a work queue.

### Usage {#work_item_example_usage}

```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
