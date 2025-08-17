# TensorFlow Lite Micro (TFLM)

The PX4 [Multicopter Neural Network](../advanced/neural_networks.md) module ([mc_nn_control](../modules/modules_controller.md#mc-nn-control)) integrates a neural network that uses the [TensorFlow Lite Micro (TFLM)](https://github.com/tensorflow/tflite-micro) inference library.

This is a mature inference library intended for use on embedded devices, and is hence a suitable choice for PX4.

This guide explains how the TFLM library is integrated into the [mc_nn_control](../modules/modules_controller.md#mc-nn-control) module, and the changes you would have to make to use it for your own neural network.

:::tip
For more information, see the [TFLM guide](https://ai.google.dev/edge/litert/microcontrollers/get_started).
:::

## TLMF NN Formats

TFLM uses networks in its own [tflite format](https://ai.google.dev/edge/litert/models/convert).
However, since many microcontrollers do not have native filesystem support, a tflite file can be converted to a C++ source and header file.

This is what is done in `mc_nn_control`.
The tflight neural network is represented in code by the files [`control_net.cpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_nn_control/control_net.cpp) and [`control_net.hpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_nn_control/control_net.hpp).

### Getting a Network in tflite Format

There are many online resource for generating networks in the `.tflite` format.

For this example we trained the network in the open source [Aerial Gym Simulator](https://ntnu-arl.github.io/aerial_gym_simulator/).
Aerial Gym includes a guide, and supports RL both for control and vision-based navigation tasks.

The project includes conversion code for `PyTorch -> TFLM` in the [resources/conversion](https://github.com/ntnu-arl/aerial_gym_simulator/tree/main/resources/conversion) folder.

### Updating `mc_nn_control` with your own NN

You can convert a `.tflite` network into a `.cc` file in the ubuntu terminal with this command:

```sh
xxd -i converted_model.tflite > model_data.cc
```

You will then have to modify the `control_net.hpp` and `control_net.cpp` to include the data from `model_data.cc`:

- Take the size of the network in the bottom of the `.cc` file and replace the size in `control_net.hpp`.
- Take the data in the model array in the `cc` file, and replace the ones in `control_net.cpp`.

You are now ready to run your own network.

## Code Explanation

This section explains the code used to integrate the NN in `control_net.cpp`.

### Operations and Resolver

Firstly we need to create the resolver and load the needed operators to run inference on the NN.
This is done in the top of `mc_nn_control.cpp`.
The number in `MicroMutableOpResolver<3>` represents how many operations you need to run the inference.

A full list of the operators can be found in the [micro_mutable_op_resolver.h](https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/micro/micro_mutable_op_resolver.h) file.
There are quite a few supported operators, but you will not find the most advanced ones.
In the control example the network is fully connected so we use `AddFullyConnected()`.
Then the activation function is ReLU, and we `AddAdd()` for the bias on each neuron.

### Interpreter

In the `InitializeNetwork()` we start by setting up the model that we loaded from the source and header file.
Next is to set up the interpreter, this code is taken from the TFLM documentation and is thoroughly explained there.
The end state is that the `_control_interpreter` is set up to later run inference with the `Invoke()` member function.
The `_input_tensor` is also defined, it is fetched from `_control_interpreter->input(0)`.

### Вхідні дані

The `_input_tensor` is filled in the `PopulateInputTensor()` function.
`_input_tensor` works by accessing the `->data.f` member array and fill in the required inputs for your network.
The inputs used in the control network is covered in [Neural Networks](../advanced/neural_networks.md).

### Виводи

For the outputs the approach is fairly similar to the inputs.
After setting the correct inputs, calling the `Invoke()` function the outputs can be found by getting `_control_interpreter->output(0)`.
And from the output tensor you get the `->data.f` array.
