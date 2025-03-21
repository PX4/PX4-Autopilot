# Neural Networks

::: warning
This is an experimental module. All flying at your own risk.
:::

There are several reasons you might want to use neural networks inside of PX4, this documentation together with an example module will help you to get started with this. The code is made to run directly on an embedded flight controller (FCU), but it can easily be modified to run on a more powerful companion computer as well.

The example module only handles inference as of now, so you will need to train the network in another framework and import it here. You can find a guide for how this was done together with the open-source software [Aerial Gym Simulator](https://ntnu-arl.github.io/aerial_gym_simulator/) for this example module. Aerial gym supports RL both for control and perception.

This page toghether with the subpages explain how the example module works, both in terms of PX4 specific code and TFLM specific code. By looking through these pages the goal is for you to quickly be able to shape the model to your needs and make your own experimental NN modules.

## Inference library

First of all we need a way to run inference on the neural network. In this example implementation TensorFlow Lite Micro ([TFLM](https://github.com/tensorflow/tflite-micro)) is used, but there are several other possibilities, like [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) and [Executorch](https://pytorch.org/executorch-overview). Do note however that importing new libraries into PX4 can be quite cumbersome.

TFLM already has support for several architectures, so there is a high likelihood that you can build it for the board you want to use. The build is already tested for three configurations and can be created with the following commands:

   ```sh
   make px4_sitl_neural
   ```

   ```sh
   make px4_fmu-v6c_neural
   ```

   ```sh
   make mro_pixracerpro_neural
   ```

:::tip
If you have a board you want to test neural control on, which is supported by px4, but not part of the examples: got to boards/"your board" and add "CONFIG_MODULES_MC_NN_CONTROL=y" to your .px4board file
:::


## Neural Control
Nerual networks can be used for a broad range of implementations, like estimation and computer vision, in the example module it is used to replace the [controllers](../flight_stack/controller_diagrams.md). In the controller diagram you can see the uORB message flow where you can replace whatever by subscribing to the previous message and publishing the next. More about [uORB](../middleware/uorb.md) can be read here. And you also need to stop the module publishing the topic you want to replace, this is covered in [NN Module Utilities](nn_module_utilities.md)

The module is called mc_nn_control and replaces the entire controller structure as well as the control allocator.

## Input
The input can be changed to whatever you want. Set ut the input you want to use during training and then provide the same input in PX4. In the Neural Control module the input is an array of 15 numbers, and consists of these values in this order:
 - [3] Local position error. (goal position - current position)
 - [6] The first 2 rows of a 3 dimentional rotation matrix.
 - [3] Linear velocity
 - [3] Angular velocity

 All the input values are collected from uORB topics and transformed into the correct representation in the PopulateInputTensor() function. PX4 uses the NED frame representation, while the Aerial Gym Simulator, in which the NN was trained, uses the ENU representation. Therefore two rotation matrices are created in the function and all the inputs are transformed from the NED representation to the ENU one.

 ![ENU-NED](../../assets/advanced/ENU-NED.png)

 ENU and NED are just rotation representations, the translational difference is only there so both can be seen in the same figure.

## Output
The output consists of 4 values, the motor forces, one for each motor. These are transformed in the RescaleActions() function. This is done because PX4 expects normalized motor commands while the Aerial Gym Simulator uses physical values. So the output from the network needs to be normalized before they can be sent to the motors in PX4. The network is currently trained for a drone platform used in the [Autonomous Robots Lab (ARL)](https://www.autonomousrobotslab.com/) at NTNU. But the controller is somewhat robust, so it could work directly on other platforms, but performing system identification and training a new network is recommended. You can find instructions for this in the [Aerial Gym Documentation](TODO).

 And then the commands are published to the [ActuatorMotors](../msg_docs/ActuatorMotors.md) topic. The reordering and the publishing is handled in PublishOutput(float* command_actions) function.

 ## Training your own network
 Since the Aerial Gym Simulator is open-source you can download it and train your own networks as long as you have access to an NVIDIA GPU. If you want to train a control network optimized for your platform you can use the configuration used in [this example](TODO). You need one system identification flight for this and an approximate inertia matrix for your platform. On the sys-id flight you need ESC telemetry, you can read more about that in [DSHOT](../peripherals/dshot.md). Then do the following steps.

 - Do a hover flight
 - Read of the logs what RPM is required for the drone to hover.
 - Use the weight of each motor, length of the motor arms, total weight of the platform with battery to calculate an approximate inertia matrix for the platform.
 - Insert these values into the Aerial Gym configuration and train your network.
 - Convert the network as explained in [TFLM](tflm.md)
