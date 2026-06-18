# Copyright © 2023-2024 Apple Inc.

import math
import time
from functools import partial

import datasets
import mlx.core as mx
import mlx.nn as nn
import mlx.optimizers as optim
import numpy as np
from mlx.utils import tree_flatten

from dataset_mlx import create_dataset


class TransformerLM(nn.Module):
    def __init__( self, vocab_size, num_layers, dims, num_heads, checkpoint: bool,):
        super().__init__()

        self.embedding = nn.Embedding(vocab_size, dims)
        self.pe = nn.SinusoidalPositionalEncoding(dims)
        self.transformer = nn.TransformerEncoder(
            num_layers, dims, num_heads, norm_first=True, checkpoint=checkpoint
        )
        self.out_proj = nn.Linear(dims, vocab_size)

    def __call__(self, x):
        L = x.shape[1]
        mask = nn.MultiHeadAttention.create_additive_causal_mask(L)
        x = self.embedding(x)
        x = x + self.pe(mx.arange(L))
        x = self.transformer(x, mask)
        return self.out_proj(x)

class GRURNN(nn.Module):
    def __init__(self, vocab_size, embedding_dim, hidden_dim, output_dim):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embedding_dim)
        self.gru = nn.GRU(embedding_dim, hidden_dim)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def __call__(self, text):
        embedded = self.embedding(text)
        # packed_embedded = nn.utils.rnn.pack_padded_sequence(embedded, text_lengths.cpu(), batch_first=True, enforce_sorted=False)
        gru_output = self.gru(embedded)
        output = self.fc(gru_output)
        return output

def iterate_batches(dataset, batch_size):
    inputs, targets = dataset
    s = 0
    while True:
        if s == 0:
            # Reset permutation:
            perm = np.random.permutation(inputs.shape[0])
        ids = perm[s : s + batch_size]
        yield inputs[ids], targets[ids]
        s += batch_size
        if s >= inputs.shape[0]:
            s = 0

sequence_length = 64
dataset = create_dataset("enwik8", sequence_length)

batch_size = 32
model = GRURNN(256, embedding_dim=32, hidden_dim=64, output_dim=256)
mx.eval(model.parameters())
nparams = sum(
    x.size for k, x in tree_flatten(model.parameters()) if "embedding" not in k
)
print(f"Training a GRU with {nparams / 1024**2:.3f} M parameters")

def loss_fn(model, x, y, reduce=True):
    logits = model(x)
    losses = nn.losses.cross_entropy(logits, y)
    return mx.mean(losses) if reduce else mx.mean(losses, axis=(-1, -2))

optimizer = optim.Adam(learning_rate=3e-4)

state = [model.state, optimizer.state]

@partial(mx.compile, inputs=state, outputs=state)
def step(inputs, targets):
    loss_and_grad_fn = nn.value_and_grad(model, loss_fn)
    loss, grads = loss_and_grad_fn(model, inputs, targets)
    optimizer.update(model, grads)
    return loss

steps_per_report = 100
num_iters = 100000
train_iterator = iterate_batches(batch_size)
losses = []
tic = time.perf_counter()
for it, (inputs, targets) in enumerate(train_iterator):
    inputs, targets = map(mx.array, (inputs, targets))
    loss = step(inputs, targets)
    mx.eval(state)
    losses.append(loss.item())
    if (it + 1) % steps_per_report == 0:
        train_loss = np.mean(losses)
        toc = time.perf_counter()
        print(
            f"Iter {it + 1}: Train loss {train_loss:.3f}, "
            f"It/sec {steps_per_report / (toc - tic):.3f}"
        )
        losses = []
        tic = time.perf_counter()
