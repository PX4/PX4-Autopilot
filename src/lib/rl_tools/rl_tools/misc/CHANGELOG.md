# Changes to latest release

- Optimizers need to be initialized now (in addition to malloc) because the default initialization of the parameters did sometimes not work on GPUs. Hence, the parameters are contained in a Tensor now for best CPU <-> GPU transfer compatiblility