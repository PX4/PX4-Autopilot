# 모듈 참고: 템플릿

## mc_raptor

Source: [modules/mc_raptor](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_raptor)

### 설명

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

### 설명

제공된 모듈 기능을 설명하는 섹션입니다.

시작/중지/상태 기능이 있는 백그라운드에서 작업으로 실행되는 모듈의 템플릿입니다.

### 구현

이 모듈의 상위 수준 구현을 설명하는 섹션입니다.

### 예

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

### 설명

작업 대기열에서 실행되는 간단한 모듈의 예입니다.

### Usage {#work_item_example_usage}

```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
