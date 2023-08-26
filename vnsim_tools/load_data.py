# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import os


def read_edge_cost(path):
    """Input:
    e1:c1
    e2:c2
    ...
    Output:
    dict{ e [N] --> c [R] }
    """
    d = {}
    with open(path) as f:
        for line in f:
            e, c = line.strip().split(':')
            c = c.split(";")[0]
            if e in d:
                print(f"In [read_edge_cost] from {path}, edge [{e}] exits multiple times.")
                continue
            e, c = int(e), float(c)
            d[e] = c
    return d

def read_edge_to_edgelist(path):
    """Input:
    e1:[o1,o2,...]
    e2:[o1',o2',...]
    ...
    Output:
    dict{ e [N] --> [o1,o2,...] [N^k] }
    """
    d = {}
    with open(path) as f:
        for line in f:
            e, o_list = line.strip().split(':')
            if e in d:
                print(f"In [read_edge_to_edgelist] from {path}, edge [{e}] exits multiple times.")
                continue
            o_list = o_list.split(";")
            e = int(e)
            d[e] = []
            for o in o_list[:-1]:
                d[e].append(int(o))
    return d

def read_map(map_path):
    E_cost_dict = read_edge_cost(os.path.join(map_path, "path.txt"))
    E_in_dict = read_edge_to_edgelist(os.path.join(map_path, "InEdges.txt"))
    E_out_dict = read_edge_to_edgelist(os.path.join(map_path, "OutEdges.txt"))
    E_clash_dict = read_edge_to_edgelist(os.path.join(map_path, "ClashedEdges.txt"))
    V_E_clash_dict = read_edge_to_edgelist(os.path.join(map_path, "EndClashedEdges.txt"))
    V2V_E_clash_dict = read_edge_to_edgelist(os.path.join(map_path, "EndEndClashedEdges.txt"))
    return {
        "E_cost": E_cost_dict,
        "E_in": E_in_dict,
        "E_out": E_out_dict,
        "E_clash": E_clash_dict,
        "V_E_clash": V_E_clash_dict,
        "V2V_E_clash": V2V_E_clash_dict,
    }
