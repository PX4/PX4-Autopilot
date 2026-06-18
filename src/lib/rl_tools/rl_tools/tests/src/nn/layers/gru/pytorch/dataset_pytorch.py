from torch.nn.utils.rnn import pad_sequence
from torch.utils.data import Dataset, DataLoader
import torch
import functools
from dataset import load_dataset
from tqdm import tqdm
class TextDataset(Dataset):
    def __init__(self, data, sequence_length):
        self.raw_data = data
        self.max_length = sequence_length + 1
        print("Preprocessing the data")
        self.data = torch.tensor(list(data), dtype=torch.int64)

    def __len__(self):
        return len(self.data) - self.max_length

    def __getitem__(self, idx):
        full_seq = self.data[idx:idx+self.max_length]
        return full_seq[:-1], full_seq[1:]

def create_dataset(dataset_name, sequence_length):
    chunks = load_dataset(dataset_name, sequence_length)
    return TextDataset(chunks, sequence_length)