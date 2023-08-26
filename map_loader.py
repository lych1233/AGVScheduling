# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import numpy as np

from c_modules.graph import Graph
from c_modules.path_table import PathTable


class MapWithClash:
    def __init__(self, data):
        self.n, self.m = len(data["vertex_position"]), len(data["edge_length"])
        self.v_locations = np.zeros((self.n + 1, 2))
        for i in range(1, self.n + 1):
            self.v_locations[i] = data["vertex_position"][str(i)]
        self.e_x, self.e_y, self.e_z = np.zeros(self.m + 1, dtype=np.int32), np.zeros(self.m + 1, dtype=np.int32), np.zeros(self.m + 1)
        for i in range(1, self.m + 1):
            x, y, z = data["edge_length"][str(i)]
            self.e_x[i] = x
            self.e_y[i] = y
            self.e_z[i] = z
        self.clash = {"vv": {}, "ve": {}, "ev": {}, "ee": {}}
        for v in range(1, self.n + 1):
            v_str = str(v)
            self.clash["vv"][v] = []
            self.clash["ve"][v] = []
            if v_str not in data["vertex_clash"]:
                continue
            for x, y in data["vertex_clash"][v_str]:
                if x == "V":
                    self.clash["vv"][v].append(y)
                else:
                    self.clash["ve"][v].append(y)
        for e in range(1, self.m + 1):
            e_str = str(e)
            self.clash["ev"][e] = []
            self.clash["ee"][e] = []
            if e_str not in data["edge_clash"]:
                continue
            for x, y in data["edge_clash"][e_str]:
                if x == "V":
                    self.clash["ev"][e].append(y)
                else:
                    self.clash["ee"][e].append(y)
        for v in range(1, self.n + 1):
            self.clash["vv"][v].sort()
            self.clash["ve"][v].sort()
        for e in range(1, self.m + 1):
            self.clash["ev"][e].sort()
            self.clash["ee"][e].sort()

    def generate_graph(self):
        return Graph(self.n, self.m, self.e_x, self.e_y, self.e_z)

    def generate_path_table(self):
        path_table = PathTable(self.n, self.m)
        for i in range(1, self.n + 1):
            v_clash = np.array(self.clash["vv"][i]).astype(np.int32)
            e_clash = np.array(self.clash["ve"][i]).astype(np.int32)
            path_table.init_clash("vertex", i, len(v_clash), v_clash, len(e_clash), e_clash)
        for i in range(1, self.m + 1):
            v_clash = np.array(self.clash["ev"][i]).astype(np.int32)
            e_clash = np.array(self.clash["ee"][i]).astype(np.int32)
            path_table.init_clash("edge", i, len(v_clash), v_clash, 0, e_clash)
        return path_table
