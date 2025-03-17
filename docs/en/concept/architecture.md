# PX4 Architectural Overview

PX4 consists of two main layers: the [flight stack](#flight-stack) is an estimation and flight control system,
and the [middleware](#middleware) is a general robotics layer that can support any type of autonomous robot, providing internal/external communications and hardware integration.

All PX4 [airframes](../airframes/index.md) share a single codebase (this includes other robotic systems like boats, rovers, submarines etc.). The complete system design is [reactive](http://www.reactivemanifesto.org), which means that:

- All functionality is divided into exchangeable and reusable components
- Communication is done by asynchronous message passing
- The system can deal with varying workload

<a id="architecture"></a>

## High-Level Software Architecture

The diagram below provides a detailed overview of the building blocks of PX4.
The top part of the diagram contains middleware blocks, while the lower
section shows the components of the flight stack.

![PX4 Architecture](../../assets/diagrams/PX4_Architecture.svg)

<!-- This diagram can be updated from
[here](https://drive.google.com/file/d/0B1TDW9ajamYkaGx3R0xGb1NaeU0/view?usp=sharing)
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

The source code is split into self-contained modules/programs (shown in `monospace` in the diagram).
Usually a building block corresponds to exactly one module.

:::tip
At runtime, you can inspect which modules are executed with the `top` command in shell, and each module can be started/stopped individually via `<module_name> start/stop`.
While `top` command is specific to NuttX shell, the other commands can be used in the SITL shell (pxh>) as well.
For more information about each of these modules see the [Modules & Commands Reference](../modules/modules_main.md).
:::

The arrows show the information flow for the _most important_ connections between the modules.
In reality, there are many more connections than shown, and some data (e.g. for parameters) is accessed by most of the modules.

Modules communicate with each other through a publish-subscribe message bus named [uORB](../middleware/uorb.md).
The use of the publish-subscribe scheme means that:

- The system is reactive — it is asynchronous and will update instantly when new data is available
- All operations and communication are fully parallelized
- A system component can consume data from anywhere in a thread-safe fashion

::: info
This architecture allows every single one of these blocks to be rapidly and easily replaced, even at runtime.
:::

### Flight Stack

The flight stack is a collection of guidance, navigation and control algorithms for autonomous drones.
It includes controllers for fixed-wing, multirotor and VTOL airframes as well as estimators for attitude and position.

The following diagram shows an overview of the building blocks of the flight stack.
It contains the full pipeline from sensors, RC input and autonomous flight control (Navigator), down to the motor or servo control (Actuators).

![PX4 High-Level Flight Stack](../../assets/diagrams/PX4_High-Level_Flight-Stack.svg)

<!-- This diagram can be updated from
[here](https://drive.google.com/a/px4.io/file/d/15J0eCL77fHbItA249epT3i2iOx4VwJGI/view?usp=sharing)
and opened with draw.io Diagrams. You might need to request access if you
don't have a px4.io Google account.
Caution: it can happen that after exporting some of the arrows are wrong. In
that case zoom into the graph until the arrows are correct, and then export
again. -->

An **estimator** takes one or more sensor inputs, combines them, and computes a vehicle state (for example the attitude from IMU sensor data).

A **controller** is a component that takes a setpoint and a measurement or estimated state (process variable) as input.
Its goal is to adjust the value of the process variable such that it matches the setpoint.
The output is a correction to eventually reach that setpoint.
For example the position controller takes position setpoints as inputs, the process variable is the currently estimated position, and the output is an attitude and thrust setpoint that move the vehicle towards the desired position.

A **mixer** takes force commands (such as "turn right") and translates them into individual motor commands, while ensuring that some limits are not exceeded.
This translation is specific for a vehicle type and depends on various factors, such as the motor arrangements with respect to the center of gravity, or the vehicle's rotational inertia.

<a id="middleware"></a>

### Middleware

The [middleware](../middleware/index.md) consists primarily of device drivers for embedded sensors, communication with the external world (companion computer, GCS, etc.) and the uORB publish-subscribe message bus.

In addition, the middleware includes a [simulation layer](../simulation/index.md) that allows PX4 flight code to run on a desktop operating system and control a computer modeled vehicle in a simulated "world".

## Update Rates

Since the modules wait for message updates, typically the drivers define how fast a module updates.
Most of the IMU drivers sample the data at 1kHz, integrate it and publish with 250Hz.
Other parts of the system, such as the `navigator`, don't need such a high update rate, and thus run considerably slower.

The message update rates can be [inspected](../middleware/uorb.md) in real-time on the system by running `uorb top`.

<a id="runtime-environment"></a>

## Runtime Environment

PX4 runs on various operating systems that provide a POSIX-API (such as Linux, macOS, NuttX or QuRT).
It should also have some form of real-time scheduling (e.g. FIFO).

The inter-module communication (using [uORB](../middleware/uorb.md)) is based on shared memory.
The whole PX4 middleware runs in a single address space, i.e. memory is shared between all modules.

::: info
The system is designed such that with minimal effort it would be possible to run each module in separate address space (parts that would need to be changed include `uORB`, `parameter interface`, `dataman` and `perf`).
:::

There are 2 different ways that a module can be executed:

- **Tasks**: The module runs in its own task with its own stack and process priority.
- **Work queue tasks**: The module runs on a shared work queue, sharing the same stack and work queue thread priority as other modules on the queue.

  - All the tasks must behave co-operatively as they cannot interrupt each other.
  - Multiple _work queue tasks_ can run on a queue, and there can be multiple queues.
  - A _work queue task_ is scheduled by specifying a fixed time in the future, or via uORB topic update callback.

  The advantage of running modules on a work queue is that it uses less RAM, and potentially results in fewer task switches.
  The disadvantages are that _work queue tasks_ are not allowed to sleep or poll on a message, or do blocking IO (such as reading from a file).
  Long-running tasks (doing heavy computation) should potentially also run in a separate task or at least a separate work queue.

::: info
Tasks running on a work queue do not show up in [`top`](../modules/modules_command.md#top) (only the work queues themselves can be seen - e.g. as `wq:lp_default`).
Use [`work_queue status`](../modules/modules_system.md#work-queue) to display all active work queue items.
:::

### Background Tasks

`px4_task_spawn_cmd()` is used to launch new tasks (NuttX) or threads (POSIX - Linux/macOS) that run independently from the calling (parent) task:

```cpp
independent_task = px4_task_spawn_cmd(
    "commander",                    // Process name
    SCHED_DEFAULT,                  // Scheduling type (RR or FIFO)
    SCHED_PRIORITY_DEFAULT + 40,    // Scheduling priority
    3600,                           // Stack size of the new task or thread
    commander_thread_main,          // Task (or thread) main function
    (char * const *)&argv[0]        // Void pointer to pass to the new task
                                    // (here the commandline arguments).
    );
```

### OS-Specific Information

#### NuttX

[NuttX](https://nuttx.apache.org//) is the primary RTOS for running PX4 on a flight-control board.
It is open source (BSD license), light-weight, efficient and very stable.

Modules are executed as tasks: they have their own file descriptor lists, but they share a single address space.
A task can still start one or more threads that share the file descriptor list.

Each task/thread has a fixed-size stack, and there is a periodic task which checks that all stacks have enough free space left (based on stack coloring).

#### Linux/macOS

On Linux or macOS, PX4 runs in a single process, and the modules run in their own threads (there is no distinction between tasks and threads as on NuttX).
