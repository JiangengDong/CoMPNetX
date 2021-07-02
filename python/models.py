#!/usr/bin/env python3
import torch
import torch.nn as nn
import numpy as np


class VoxelEncoder(nn.Module):
    def __init__(self, input_size: int, output_size: int):
        super(VoxelEncoder, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(in_channels=input_size, out_channels=16, kernel_size=[5, 5], stride=[2, 2]),
            nn.PReLU(),
            nn.Conv2d(in_channels=16, out_channels=8, kernel_size=[3, 3], stride=[1, 1]),
            nn.PReLU(),
            nn.MaxPool2d(kernel_size=[2, 2])
        )
        x = self.encoder(torch.autograd.Variable(torch.rand([1, input_size, input_size, input_size])))
        first_fc_in_features = np.prod(x.shape[1:])
        self.head = nn.Sequential(
            nn.Linear(first_fc_in_features, 256),
            nn.PReLU(),
            nn.Linear(256, output_size)
        )

    def forward(self, x):
        x = self.encoder(x)
        x = x.view(x.shape[0], -1)
        x = self.head(x)
        return x


class PNet(nn.Module):
    def __init__(self, input_size, output_size):
        super(PNet, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, 896), nn.PReLU(), nn.Dropout(),
            nn.Linear(896, 512), nn.PReLU(), nn.Dropout(),
            nn.Linear(512, 256), nn.PReLU(), nn.Dropout(),
            nn.Linear(256, 64), nn.PReLU(),
            nn.Linear(64, output_size))

    def forward(self, x):
        out = self.fc(x)
        return out


class EnetConstraint(nn.Module):
    def __init__(self, input_size: int, output_size: int):
        super(EnetConstraint, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, 128), nn.PReLU(),  # 128
            nn.Linear(128, output_size)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.fc(x)
        return out


class DNet(nn.Module):
    def __init__(self, input_size: int, output_size: int):
        super(DNet, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, 256), nn.PReLU(),
            nn.Linear(256, 256), nn.PReLU(),
            nn.Linear(256, output_size))

    def forward(self, x):
        out = self.fc(x)
        return out
