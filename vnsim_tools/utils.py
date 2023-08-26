# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import json

class RawMyDict():
    def __init__(self):
        self.raw_to_my_dict = {}
        self.my_to_raw_dict = {}
        self.cnt = 0
        self.freezing = False

    def add_dict(self, x):
        self.cnt += 1
        self.raw_to_my_dict[x] = self.cnt
        self.my_to_raw_dict[self.cnt] = x

    def raw_to_my(self, x):
        if x == -1:
            return -1
        if x not in self.raw_to_my_dict:
            if self.freezing:
                raise ValueError(f"{x} is not a known ID (raw)")
            self.add_dict(x)
        return self.raw_to_my_dict[x]

    def my_to_raw(self, x):
        if x == -1:
            return -1
        if x not in self.my_to_raw_dict:
            raise ValueError(f"{x} is not a known ID (my)")
        return self.my_to_raw_dict[x]

    def total_entries(self):
        return self.cnt

    def freeze(self):
        self.freezing = True

    def unfreeze(self):
        self.freezing = False
    
    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.raw_to_my_dict, f)
    
    def load(self, path):
        self.raw_to_my_dict = {}
        self.my_to_raw_dict = {}
        with open(path, "r") as f:
            raw_to_my_dict = json.load(f)
        for k in raw_to_my_dict:
            self.raw_to_my_dict[int(k)] = int(raw_to_my_dict[k])
        for k, v in self.raw_to_my_dict.items():
            self.my_to_raw_dict[v] = k
