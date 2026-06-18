import json
import zipfile
import gzip
import os
# from torchtext.vocab import build_vocab_from_iterator
from tqdm import tqdm


def load_dataset(dataset_name, sequence_length, overlapping_sequences=False):
    # Load the data
    print("Loading the data:")
    if dataset_name == "enwik8_small":
        file_name = "enwik8.small.zip"
    elif dataset_name == "enwik8":
        file_name = "enwik8.zip"
    else:
        file_name = "00c2bfc7-57db-496e-9d5c-d62f8d8119e3.json.zip"
    if os.path.exists("/Users"):
        full_data_path = "/Users/jonas/Downloads/" + file_name
    else:
        full_data_path = "/home/jonas/Downloads/" + file_name

    if dataset_name == "enwik8_small":
        full_data = None
        with gzip.open(full_data_path, "r") as z:
            full_data = z.read()
    elif dataset_name == "enwik8": 
        full_data = None
        with zipfile.ZipFile(full_data_path, "r") as z:
            with z.open(os.path.basename(full_data_path)[:-4]) as f:
                full_data = f.read()
    else:
        full_data = None
        with zipfile.ZipFile(full_data_path, "r") as z:
            with z.open(os.path.basename(full_data_path)[:-4]) as f:
                full_data = json.load(f)
    print(f"Full data size: {len(full_data)}")

    n_articles = 100

    if dataset_name.startswith("enwik8"):
        long_text = full_data
    else:
        print("Concatenating the dataset")
        def get_texts(data):
            for item in tqdm(data[:n_articles]):
                yield item['text']
        long_text = "\n".join(get_texts(full_data)).encode("utf-8")
    print(f"Long text size: {len(long_text)}")

    # print("Chunking the dataset")
    # max_length = sequence_length + 1
    # chunks = [long_text[i:i + max_length] for i in tqdm(range(0, len(long_text)-max_length, 1 if overlapping_sequences else sequence_length))]
    # return chunks
    return long_text