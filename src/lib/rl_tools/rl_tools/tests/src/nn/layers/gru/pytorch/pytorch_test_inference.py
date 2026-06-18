import torch
import os
import lightning as pl
from torch.nn.utils.rnn import pad_sequence

from model import model

import difflib

def color_diff(a, b):
    matcher = difflib.SequenceMatcher(None, a, b)
    result = []
    for opcode, a0, a1, b0, b1 in matcher.get_opcodes():
        if opcode == 'equal':
            result.append(a[a0:a1])
        elif opcode == 'insert':
            result.append(f"\033[92m{b[b0:b1]}\033[0m")  # Green for insertions
        elif opcode == 'delete':
            result.append(f"\033[91m{a[a0:a1]}\033[0m")  # Red for deletions
        elif opcode == 'replace':
            result.append(f"\033[91m{a[a0:a1]}\033[0m\033[92m{b[b0:b1]}\033[0m")
    return ''.join(result)

def load_last_checkpoint(model, checkpoint_dir, checkpoint_path=None):
    if checkpoint_path is None:
        checkpoints = [f for f in os.listdir(checkpoint_dir) if f.endswith('.ckpt')]
        if not checkpoints:
            raise ValueError("No checkpoints found in the specified directory.")
        latest_checkpoint = max(checkpoints, key=lambda x: os.path.getctime(os.path.join(checkpoint_dir, x)))
        checkpoint_path = os.path.join(checkpoint_dir, latest_checkpoint)
    print(f"Loading checkpoint from {checkpoint_path}")
    checkpoint = torch.load(checkpoint_path, map_location=torch.device('cpu'))
    model.load_state_dict(checkpoint['state_dict'])
    return model

def generate_text(model, prompt, max_length=100, n=100):
    model.eval()
    device = next(model.parameters()).device
    tokens = list(bytes(prompt, 'utf-8'))
    input_tensor = torch.tensor(tokens, dtype=torch.int64).unsqueeze(0).to(device)
    input_lengths = torch.tensor([len(tokens)])

    generated_tokens = tokens.copy()
    
    with torch.no_grad():
        for _ in range(n):
            output = model(input_tensor[:1, -max_length:])
            temperature = 0.5
            next_token_idx = torch.multinomial(torch.softmax(output[0, -1] / temperature, dim=0), 1).item()
            generated_tokens.append(next_token_idx)
            input_tensor = torch.cat([input_tensor, torch.tensor([[next_token_idx]]).to(device)], dim=1)
            input_lengths = torch.tensor([input_tensor.size(1)])

    generated_text = bytes(generated_tokens).decode("utf-8")
    return generated_text

def main():
    global model
    # Load the last checkpoint
    checkpoint_dir = "checkpoints/"
    checkpoint_path = None
    model = load_last_checkpoint(model, checkpoint_dir, checkpoint_path=checkpoint_path)

    # Move model to GPU if available
    device = torch.device("cpu")
    model = model.to(device)

    # Get user input for the prompt
    # prompt = input("Enter a prompt for text generation: ")
    # prompt = "signation was decommissioned in the process; signage was removed by August 2020 to reflect the chang"
    # prompt = "and"

    prompt = "The car was on the stre"

    # Generate text
    generated_text = generate_text(model, prompt, max_length=100, n=1000)

    print("Generated text:")
    print(generated_text)
    if False:
        print(color_diff(long_text, generated_text))

if __name__ == "__main__":
    main()
