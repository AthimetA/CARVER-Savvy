#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas

# original implementation from: 
# https://github.com/tomasvr/turtlebot3_drlnav
# https://github.com/ailabspace/drl-based-mapless-crowd-navigation-with-perceived-risk
# 

# Modified by:  Athimet Aiewcharoen     , FIBO, KMUTT
#               Tanut   Bumrungvongsiri , FIBO, KMUTT
# Date : 2024-05-26

import numpy as np
import random
from collections import deque
import itertools

from settings.constparams import BUFFER_SIZE


class ReplayBuffer:
    def __init__(self, size=BUFFER_SIZE):
        self.max_size = size
        self.buffer = deque(maxlen=self.max_size)

    def sample(self, batchsize):
        batch = []
        batchsize = min(batchsize, self.get_length())
        batch = random.sample(self.buffer, batchsize)
        s_array = np.float32([array[0] for array in batch])
        a_array = np.float32([array[1] for array in batch])
        r_array = np.float32([array[2] for array in batch])
        new_s_array = np.float32([array[3] for array in batch])
        done_array = np.float32([array[4] for array in batch])

        return s_array, a_array, r_array, new_s_array, done_array

    def get_length(self):
        return len(self.buffer)

    def add_sample(self, s, a, r, new_s, done):
        transition = (s, a, r, new_s, done)
        self.buffer.append(transition)
