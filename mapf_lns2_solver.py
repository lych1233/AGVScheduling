# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
import numpy as np

from map_loader import MapWithClash
from c_modules.sipp import SIPP
from c_modules.mapf_lns2 import LNS


class MAPF_LNS2_Solver:
    def __init__(self, args, data):
        self.map = MapWithClash(data)
        self.AGV = len(data["agv_init_vertex"])
        self.graph = self.map.generate_graph()
        self.graph.compute_pairwise_distance()
        self.path_table = self.map.generate_path_table()
        self.sipp = SIPP(self.map.n, self.map.m, max_time=args.sipp_max_time, safe_eps=args.sipp_safe_eps)
        self.sipp.set_inner_graph(self.graph)
        self.sipp.set_inner_path_table(self.path_table)
        self.lns = LNS(self.map.n, self.map.m, self.AGV, self.graph, self.path_table, self.sipp)
        self.lns.seed(args.seed)
        if args.mapf_lns_max_steps > 0:
            self.lns.set_max_replan_steps(args.mapf_lns_max_steps)
        if args.mapf_lns_max_time > 0:
            self.lns.set_max_replan_time(args.mapf_lns_max_time)
    
    def MAPF(self, start_vertices, start_edges, goal_vertices, start_endpoints, start_time, target_stay_time):
        """Start from [start_vertice] or [start_edge] (the other is set to -1), to [goal_vertex] (-1 for stop at any goal_vertex);
        In MAPF, plan the path from [start_endpoint] starting at time [start_time], and staying at the target vertex for [target_stay_time]
        """
        print("run MAPF!")
        print("start_vertices", start_vertices)
        print("start_edges", start_edges)
        print("goal_vertices", goal_vertices)
        print("start_endpoints", start_endpoints)
        print("start_time", start_time)
        print("target_stay_time", target_stay_time)
        conflict_graph, solutions = self.lns.solveMAPF(
            np.array(start_vertices).astype(np.int32),
            np.array(start_edges).astype(np.int32),
            np.array(goal_vertices).astype(np.int32),
            np.array(start_endpoints).astype(np.int32),
            np.array(start_time).astype(np.float64),
            np.array(target_stay_time).astype(np.float64),
        )
        return conflict_graph, solutions
