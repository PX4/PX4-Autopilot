# Module Template for Full Applications

An application can be written to run as either a *task* (a module with its own stack and process priority) or as a *work queue task* (a module that runs on a work queue thread, sharing the stack and thread priority with other tasks on the work queue).
In most cases a work queue task can be used, as this minimizes resource usage.

::: info
[Architectural Overview > Runtime Environment](../concept/architecture.md#runtime-environment) provides more information about tasks and work queue tasks.
:::

::: info
All the things learned in the [First Application Tutorial](../modules/hello_sky.md) are relevant for writing a full application.
:::

## Work Queue Task

PX4-Autopilot contains a template for writing a new application (module) that runs as a *work queue task*: 
[src/examples/work_item](https://github.com/PX4/PX4-Autopilot/tree/main/src/examples/work_item).

A work queue task application is just the same as an ordinary (task) application, except that it needs to specify that it is a work queue task, and schedule itself to run during initialisation.

The example shows how.
In summary:
1. Specify the dependency on the work queue library in the cmake definition file ([CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/examples/work_item/CMakeLists.txt)):
   ```
   ...
   DEPENDS
      px4_work_queue
   ```
1. In addition to `ModuleBase`, the task should also derive from `ScheduledWorkItem` (included from [ScheduledWorkItem.hpp]( https://github.com/PX4/PX4-Autopilot/blob/main/platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp))
1. Specify the queue to add the task to in the constructor initialisation.
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

1. Implement the `ScheduledWorkItem::Run()` method to perform "work".
1. Implement the `task_spawn` method, specifying that the task is a work queue (using the `task_id_is_work_queue` id.
1. Schedule the work queue task using one of the scheduling methods (in the example we use `ScheduleOnInterval` from within the `init` method).



## Tasks

PX4/PX4-Autopilot contains a template for writing a new application (module) that runs as a task on its own stack:
[src/templates/template_module](https://github.com/PX4/PX4-Autopilot/tree/main/src/templates/template_module).

The template demonstrates the following additional features/aspects that are required or are useful for a full application:

- Accessing parameters and reacting to parameter updates.
- uORB subscriptions and waiting for topic updates.
- Controlling the task that runs in the background via `start`/`stop`/`status`.
  The `module start [<arguments>]` command can then be directly added to the
  [startup script](../concept/system_startup.md).
- Command-line argument parsing.
- Documentation: the `PRINT_MODULE_*` methods serve two purposes (the API is
  documented [in the source code](https://github.com/PX4/PX4-Autopilot/blob/v1.8.0/src/platforms/px4_module.h#L381)):
  - They are used to print the command-line usage when entering `module help` on the console.
  - They are automatically extracted via script to generate the [Modules & Commands Reference](../modules/modules_main.md) page.

