# 适用于开发完整应用的模版

An application can be written to run as either a _task_ (a module with its own stack and process priority) or as a _work queue task_ (a module that runs on a work queue thread, sharing the stack and thread priority with other tasks on the work queue).
在大多数情况下，可以使用工作队列任务，因为这会减少资源的使用。

:::info
[Architectural Overview > Runtime Environment](../concept/architecture.md#runtime-environment) provides more information about tasks and work queue tasks.
:::

:::info
All the things learned in the [First Application Tutorial](../modules/hello_sky.md) are relevant for writing a full application.
:::

## 工作队列任务

PX4-Autopilot contains a template for writing a new application (module) that runs as a _work queue task_:
[src/examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/work_item).

工作队列任务应用程序与普通(任务)应用程序相同。 除了它需要指定它是一个工作队列任务，并在初始化期间运行调度它本身。

示例显示了如何操作。
总结：

1. Specify the dependency on the work queue library in the cmake definition file ([CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/work_item/CMakeLists.txt)):
  ```
  ...
  DEPENDS
     px4_work_queue
  ```

2. In addition to `ModuleBase`, the task should also derive from `ScheduledWorkItem` (included from [ScheduledWorkItem.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp))

3. 在构造函数初始化中指定要添加任务的队列。
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

## 任务

PX4/PX4-Autopilot contains a template for writing a new application (module) that runs as a task on its own stack:
[src/templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module).

该模板演示了完整应用程序所需或有用的以下附加功能/方面：

- 访问参数并对参数更新做出反应。
- 订阅、等待 topic 更新。
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  [startup script](../concept/system_startup.md).
- 命令行参数解析。
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is
  documented [in the source code](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381)):
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the [Modules & Commands Reference](../modules/modules_main.md) page.

