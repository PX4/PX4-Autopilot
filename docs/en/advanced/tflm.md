# TensorFlow Lite Micro (TFLM)

[TFLM](https://github.com/tensorflow/tflite-micro) is a mature inference library intended for use on embedded devices. It is therefore a solid library for doing inference on an FCU within PX4. This is a guide on how to use the TFLM library and the implementation in the mc_nn_control module. The TFLM guide can be found in their [docs](https://ai.google.dev/edge/litert/microcontrollers/get_started).

## Network Format
The netorks should be in the tflite format, but since many microcontrollers do not have native filesystem support the tflite file is again converted to a C++ source and header file. In the module you can see it as control_net.cpp and control_net.hpp. If you have a .tflite network you can convert it in the ubuntu terminal with this command:

```sh
xxd -i converted_model.tflite > model_data.cc
```

Then take the size of the network in the bottom of the .cc file and replace the size in control_net.hpp and take the actual numbers in the array and replace the ones in control_net.cpp. And then you are good to run your own network. To get your network in the .tflite format there are lots of resources online, for this example we trained the network in the [Aerial Gym Simulator](https://ntnu-arl.github.io/aerial_gym_simulator/) and there you can also find the conversion code for PyTorch -> TFLM.

## Operations and Resolver
Firstly we need to create the resolver and load the needed operators to run inference on the NN. This is done in the top of mc_nn_control.cpp. The number in MicroMutableOpResolver<4> represents how many operations you need to run the inference. A full list of the operators can be found in the [micro_mutable_op_resolver.h](https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/micro/micro_mutable_op_resolver.h) file. There are quite a few supported operators, but you will not find the most advanced ones. In the control example the network is fully connected so we use AddFullyConnected(). Then the activation function is TODO, and we AddAdd() for the bias on each neuron.

## Interpreter
In the InitializeNetwork() we start by setting up the model that we loaded from the source and header file. Next is to set up the interpreter, this code is taken from the TFLM documentation and is thouroughly explained there. The end state is that the _control_interpreter is set up to later run inference with the Invoke() member function. The _input_tensor is also defined, it is fetched from _control_interpreter->input(0).

## Inputs
The _input_tensor is filled in the PopulateInputTensor() function. _input_tensor works by acessing the ->data.f member array and fill inn the required inputs for your network. The inputs used in the control network is covered in [Neural Networks](../advanced/neural_networks.md).

## Outputs
For the outputs the approach is fairly similar to the inputs. After setting the correct inputs, calling the Invoke() function the outputs can be found by getting _control_interpreter->output(0). And from the output tensor you get the ->data.f array.
