import jax.numpy as jnp
import jax
import flax.linen as nn
from jax import grad, vjp
jax.config.update("jax_enable_x64", True)
import h5py
import numpy as np


class SingleStepGRU(nn.Module):
    hidden_size: int
    @nn.compact
    def __call__(self, carry, x):
        gru = nn.GRUCell(self.hidden_size)
        return gru(carry, x)

class SequenceGRU(nn.Module):
    hidden_size: int
    @nn.compact
    def __call__(self, x):
        gru = nn.RNN(nn.GRUCell(self.hidden_size, dtype=dtype, param_dtype=dtype), time_major=True)
        return gru(x)


key = jax.random.PRNGKey(0)

dtype = jnp.float64
hidden_size = 16
single_step_gru = SingleStepGRU(hidden_size=hidden_size)
sequence_gru = SequenceGRU(hidden_size=hidden_size)
batch_size = 128
sequence_length = 50
feature_dim = 1
n_batches = 10



def store_weights(weights, group, gradient=False):
    for cluster in ["h", "i"]:
        for name in ["r", "z", "n"]:
            current = cluster + name
            group.create_dataset(f"W_{current}", data=np.array(weights[current]["kernel"]).T)
            if cluster == "i" or name == "n":
                group.create_dataset(f"b_{current}", data=np.array(weights[current]["bias"]))
            elif gradient:
                group.create_dataset(f"b_{current}", data=np.array(weights["i" + name]["bias"]))
            else:
                group.create_dataset(f"b_{current}", data=np.zeros((hidden_size,)))




with h5py.File("tests/data/gru_training_trace_jax.h5", "w") as f:
    epoch_group = f.create_group(str(0))
    for batch_i in range(n_batches):
        print(f"Batch {batch_i}/{n_batches}")
        batch_group = epoch_group.create_group(str(batch_i))
        key, current_key = jax.random.split(key)
        x = jax.random.normal(key, (sequence_length, batch_size, feature_dim), dtype=dtype)
        initial_carry = jnp.zeros((batch_size, hidden_size), dtype=dtype)

        single_step_gru_variables = single_step_gru.init(current_key, initial_carry, x[0])
        single_step_gru_variables = jax.tree.map(lambda param: param.astype(dtype), single_step_gru_variables)
        sequence_gru_variables = sequence_gru.init(key, x)

        sequence_gru_variables["params"]["GRUCell_0"] = single_step_gru_variables["params"]["GRUCell_0"]

        weights_group = batch_group.create_group("weights")
        weights = sequence_gru_variables["params"]["GRUCell_0"]

        store_weights(weights, weights_group)

        carry = initial_carry
        d_inputs = []
        d_carries = []
        d_variables = []
        outputs = []
        carries = [carry]

        actual_output = jax.random.normal(current_key, (sequence_length, batch_size, hidden_size), dtype=dtype)

        def output_fn(x, target):
            return ((x - target) ** 2).sum()

        for i in range(sequence_length):
            carry, output = single_step_gru.apply(single_step_gru_variables, carry, x[i])
            outputs.append(output)
            carries.append(carry)

        carries = jnp.stack(carries)
        outputs = jnp.stack(outputs)

        d_outputs = jax.grad(lambda outputs: output_fn(outputs, actual_output))(outputs)

        d_outputs_and_carry = d_outputs.copy()

        current_d_carry = jnp.zeros((batch_size, hidden_size), dtype=dtype)

        vjpure = lambda f, x: vjp(f, x)[-1]

        for i in list(range(sequence_length))[::-1]:
            carry = carries[i]
            d_inputs.append(vjpure(lambda x: single_step_gru.apply(single_step_gru_variables, carry, x), x[i])((current_d_carry, d_outputs[i]))[0])
            d_variables.append(vjpure(lambda var: single_step_gru.apply(var, carry, x[i]), single_step_gru_variables)((current_d_carry, d_outputs[i]))[0])
            current_d_carry = vjpure(lambda carry: single_step_gru.apply(single_step_gru_variables, carry, x[i]), carry)((current_d_carry, d_outputs[i]))[0]
            d_carries.append(current_d_carry)

        d_inputs = jnp.stack(d_inputs[::-1])

        sequence_gru_outputs = sequence_gru.apply(sequence_gru_variables, x)

        d_output = grad(lambda output: output_fn(output, actual_output))(sequence_gru_outputs)
        d_variables_combined = grad(lambda var: output_fn(sequence_gru.apply(var, x), actual_output))(sequence_gru_variables)

        d_weights_step_combined = jax.tree.map(lambda *x: sum(x), *d_variables)

        diff_gradient = jax.tree_util.tree_reduce(lambda x, y: x + y, jax.tree.map(lambda x, y: jnp.abs(x - y).sum(), d_weights_step_combined, d_variables_combined))

        d_variables_cumsum = [d_variables[0]]
        for d_variables_step in d_variables[1:]:
            d_variables_cumsum.append(jax.tree.map(lambda a, b: a + b, d_variables_cumsum[-1], d_variables_step))
        
        
        backward_group = batch_group.create_group("backward")
        for step_i, d_variables_step in enumerate(d_variables_cumsum[::-1]):
            step_group = backward_group.create_group(str(step_i))
            store_weights(d_variables_step["params"]["GRUCell_0"], step_group, gradient=True)
                
        d_weights_group = batch_group.create_group("gradient")
        store_weights(d_variables_combined["params"]["GRUCell_0"], d_weights_group, gradient=True)


        diff = jnp.abs(sequence_gru_outputs - outputs).sum()/jnp.array(outputs.shape).prod()

        batch_group.create_dataset("input", data=x)
        batch_group.create_dataset("d_input", data=d_inputs)
        batch_group.create_dataset("gru_output", data=outputs)
        batch_group.create_dataset("d_loss_d_y_pred_gru", data=d_outputs)


        print(f"diff: {diff}")



