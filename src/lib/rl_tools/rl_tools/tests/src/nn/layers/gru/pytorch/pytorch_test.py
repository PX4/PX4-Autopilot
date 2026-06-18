import torch
from lightning.pytorch.callbacks import ModelCheckpoint
from pytorch_lightning.loggers import WandbLogger
from torch.utils.data import DataLoader
import lightning as pl
import datetime

from dataset_pytorch import create_dataset
from model import model, sequence_length, batch_size



# Initialize WandbLogger
wandb_logger = WandbLogger(project='gru-test')


run_date_time =  datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
checkpoint_callback = ModelCheckpoint(
    dirpath="checkpoints/",
    filename=f"{run_date_time}-{{epoch:05d}}",
    save_top_k=-1,  # Save all checkpoints
    every_n_epochs=1  # Save a checkpoint every 2 epochs
)

# Create model and trainer

device = accelerator='gpu' if torch.cuda.is_available() else ('mps' if torch.backends.mps.is_available() else 'cpu') # mps is slower than 'cpu' on apple silicon
print(f"Using device: {device}")

trainer = pl.Trainer(max_epochs=1000, accelerator=device, devices=1, logger=wandb_logger, callbacks=[checkpoint_callback])

# Create DataLoader
dataset = create_dataset("enwik8", sequence_length)
train_loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=8)

# Train the model
trainer.fit(model, train_loader)

