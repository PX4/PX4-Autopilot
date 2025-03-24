# 전체 애플리케이션을 위한 모듈 템플릿

An application can be written to run as either a _task_ (a module with its own stack and process priority) or as a _work queue task_ (a module that runs on a work queue thread, sharing the stack and thread priority with other tasks on the work queue).
대부분은, 리소스 최소화를 위하여 작업 대기열을 사용합니다.

:::info
[Architectural Overview > Runtime Environment](../concept/architecture.md#runtime-environment) provides more information about tasks and work queue tasks.
:::

:::info
All the things learned in the [First Application Tutorial](../modules/hello_sky.md) are relevant for writing a full application.
:::

## 작업 대기열 작업

PX4-Autopilot contains a template for writing a new application (module) that runs as a _work queue task_:
[src/examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/work_item).

작업 대기열 작업 응용 프로그램은 작업 대기열 작업임을 지정하고 초기화중에 실행되도록 예약해야 한다는 점을 제외하고 일반(작업) 응용 프로그램과 동일합니다.

예제는 방법을 설명합니다.
요약

1. Specify the dependency on the work queue library in the cmake definition file ([CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/work_item/CMakeLists.txt)):
  ```
  ...
  DEPENDS
     px4_work_queue
  ```

2. In addition to `ModuleBase`, the task should also derive from `ScheduledWorkItem` (included from [ScheduledWorkItem.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp))

3. 생성자 초기화에서 작업을 추가할 대기열을 지정합니다.
  The [work_item](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/work_item/WorkItemExample.cpp#L42) example adds itself to the `wq_configurations::test1` work queue as shown below:

  ```cpp
  WorkItemExample::WorkItemExample() :
      ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
  {
  }
  ```

  ::: info
  The available work queues (`wq_configurations`) are listed in [WorkQueueManager.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L49).

:::

4. Implement the `ScheduledWorkItem::Run()` method to perform "work".

5. Implement the `task_spawn` method, specifying that the task is a work queue (using the `task_id_is_work_queue` id.

6. Schedule the work queue task using one of the scheduling methods (in the example we use `ScheduleOnInterval` from within the `init` method).

## 작업

PX4/PX4-Autopilot contains a template for writing a new application (module) that runs as a task on its own stack:
[src/templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module).

템플릿은 전체 애플리케이션에 필요하거나 유용한 다음과 같은 추가 기능/측면을 보여줍니다.

- 매개변수에 액세스하고 매개변수 업데이트에 반응합니다.
- uORB 구독 및 주제 업데이트 대기 중입니다.
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  [startup script](../concept/system_startup.md).
- 명령줄 인수 구문 분석.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is
  documented [in the source code](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381)):
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the [Modules & Commands Reference](../modules/modules_main.md) page.

