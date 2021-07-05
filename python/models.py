#!/usr/bin/env python3

"""
Copyright (c) 2020, University of California, San Diego
All rights reserved.

Author: Jiangeng Dong <jid103@ucsd.edu>
        Ahmed Qureshi <a1qureshi@ucsd.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

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
            nn.Linear(256, 128), nn.PReLU(),
            nn.Linear(128, 64), nn.PReLU(),
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
