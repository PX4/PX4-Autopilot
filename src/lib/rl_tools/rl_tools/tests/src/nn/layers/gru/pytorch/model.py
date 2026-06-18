from torch import nn
import torch
import lightning as pl

# PyTorch Lightning Module
class GRURNN(pl.LightningModule):
    def __init__(self, vocab_size, embedding_dim, hidden_dim, output_dim, num_layers=1):
        super().__init__()
        self.embedding = nn.Embedding(vocab_size, embedding_dim)
        self.gru = nn.GRU(embedding_dim, hidden_dim, batch_first=True, num_layers=num_layers)
        # self.gru = nn.LSTM(embedding_dim, hidden_dim, batch_first=True, num_layers=num_layers)
        self.down_projection = nn.Linear(hidden_dim, embedding_dim)
        self.fc = nn.Linear(embedding_dim, output_dim)

    def forward(self, text):
        embedded = self.embedding(text)
        # packed_embedded = nn.utils.rnn.pack_padded_sequence(embedded, text_lengths.cpu(), batch_first=True, enforce_sorted=False)
        gru_output, hidden = self.gru(embedded)
        down_projected = self.down_projection(gru_output)
        output = self.fc(down_projected)
        return output

    def training_step(self, batch):
        text, targets = batch
        text = text.to(self.device)
        output = self(text)
        loss = nn.functional.cross_entropy(output.reshape(-1, output.shape[-1]), targets.reshape(-1))
        perplexity = torch.exp(loss)
        self.log('train_loss', loss, prog_bar=True, on_epoch=True)
        self.log('train_perplexity', perplexity, prog_bar=True, on_epoch=True)
        return loss

    def configure_optimizers(self):
        return torch.optim.Adam(self.parameters())


model_name = "useful"

if model_name == "base":
    sequence_length = 64
    batch_size = 32
    model = GRURNN(256, embedding_dim=32, hidden_dim=64, output_dim=256, num_layers=1)
elif model_name == "useful":
    sequence_length = 256
    batch_size = 32
    model = GRURNN(256, embedding_dim=64, hidden_dim=256, output_dim=256, num_layers=1)