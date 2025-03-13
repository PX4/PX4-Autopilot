# 모듈 참고: 템플릿

## module

Source: [templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module)

### 설명

제공된 모듈 기능을 설명하는 섹션입니다.

시작/중지/상태 기능이 있는 백그라운드에서 작업으로 실행되는 모듈의 템플릿입니다.

### 구현

이 모듈의 상위 수준 구현을 설명하는 섹션입니다.

### 예

CLI 사용 예:

```
module start -f -p 42
```

<a id="module_usage"></a>

### 사용법

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

<a id="work_item_example_usage"></a>

### 사용법

```
work_item_example <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
