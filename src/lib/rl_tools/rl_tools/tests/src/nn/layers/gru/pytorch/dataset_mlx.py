import numpy as np

from dataset import max_length, chunks

def create_dataset(dataset_name, sequence_length):

    data = np.zeros((len(chunks), max_length), dtype=np.int64)
    for i in range(len(data)):
        text = chunks[i]
        data[i, :] = np.array(list(text), dtype=np.int64)
    dataset = data[:, :-1], data[:, 1:]

    return dataset