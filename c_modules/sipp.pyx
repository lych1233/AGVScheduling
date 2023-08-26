# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.algorithm cimport upper_bound
from libcpp.queue cimport priority_queue

DEF inf = 1e30


cdef class Planner:
    def plan(self, int start, int target, double start_time=0.0, double target_stay_time=0.0):
        """Find the path
            start from "start" at time "start_time",
            goto "target" and stay for "target_stay_time",
        with 1. min #collisions; 2. min time
        Output:
        (
            # collision,
            [(e_1, l_1, r_1), ... , (e_n, l_n, r_n), (-1, r_n+target_stay_time, r_n+target_stay_time) or (0, tn, tn)] s.t. e_1.v_in = start, e_n.v_out = target
        )"""
        pass

cdef class SIPP(Planner):
    def __init__(self, int n, int m, double max_time, double safe_eps):
        self.n, self.m = n, m
        self.MAX_TIME, self.SAFE_EPS = max_time, safe_eps
        self.num_intervals.resize(self.n + 1)
        self.interval_version.resize(self.n + 1, -1)
        self.edge_safe_interval_version.resize(self.m + 1, -1)
        self.interval_is_safe.resize(self.n + 1)
        self.intervals.resize(self.n + 1)
        self.edge_safe_intervals.resize(self.m + 1)
    
    def set_inner_graph(self, Graph graph):
        self.inner_edge_out = graph.edge_out
    
    def set_inner_path_table(self, PathTable path_table):
        self.v_xid.resize(self.n + 1)
        self.e_xid.resize(self.m + 1)
        cdef int i
        for i in range(1, self.n + 1):
            self.v_xid[i] = path_table.v_to_xid(i)
        for i in range(1, self.m + 1):
            self.e_xid[i] = path_table.e_to_xid(i)
    
    cdef void refresh_nearby_safe_intreval(self, int x, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version):
        cdef int xid = self.v_xid[x]
        self.update_interval(x, clash_table[xid], clash_node_version[xid])
        self.sipp_vertex_reset(x)
        cdef int y, e
        for edge_node in self.inner_edge_out[x]:
            y = edge_node.v_out
            xid = self.v_xid[y]
            self.update_interval(y, clash_table[xid], clash_node_version[xid])
            self.sipp_vertex_reset(y)
            e = edge_node.e_id
            xid = self.e_xid[e]
            self.update_edge_safe_interval(e, edge_node.cost, clash_table[xid], clash_node_version[xid])
    
    cdef void sipp_vertex_reset(self, int x):
        if self.arrival_time[0][x].size() != 0:
            return
        self.max_num_safe_intervals = max(self.max_num_safe_intervals, self.num_intervals[x])
        cdef size_t i
        for i in range(self.arrival_time.size()):
            self.arrival_time[i][x].resize(self.num_intervals[x], inf)
            self.fa[i][x].resize(self.num_intervals[x])

    cdef int kth_interval(self, int x, double t, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version):
        it = upper_bound(self.intervals[x].begin(), self.intervals[x].end(), dpair(t, self.MAX_TIME))
        cdef int k
        if it == self.intervals[x].end():
            k = self.intervals[x].size() - 1
        else:
            k = it - self.intervals[x].begin() - 1
        return k
       
    cdef sipp_process(self, int target, double target_stay_time, sipp_vec& f, sipp_vec& g, parent_vec& fa_f, parent_vec& fa_g, dvec& heuristic, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version):
        """Core function of SIPP
            for each vertex x, we have a series of intervals:
                - [0, t1), [t1, t2], [t2, t3), ..., [tn, maxT)
                - safe,    unsafe,   safe,   , ..., (can also start with unsafe)
            f, g: earlist arrival time f[x][i] in [l[x][i], r[x][i]) with f: non-collision, and g: one-collision. Only f update will be inserted to Q
            Run in O((#intervals) log N)
                - #intervals can be the number of all clashes (on vertex and edge), or the number of all intervals (on vertex and edge)
        """
        cdef priority_queue[pair[double, ipair]] Q
        cdef int x, y, e, i, j, k
        cdef double t, arrival_t_l, arrival_t_r, earliest_safe_t
        cdef vector[vector[bint]] vis
        vis.resize(self.n + 1)
        for x in range(1, self.n + 1):
            if f[x].size() == 0:
                continue
            g[x].resize(self.num_intervals[x], inf)
            fa_g[x].resize(self.num_intervals[x])
            for i in range(self.num_intervals[x]):
                if f[x][i] < inf:
                    Q.push(pair[double, ipair](-(f[x][i] + heuristic[x]), ipair(x, i)))
        cdef int temp_target = target
        cdef int temp_interval = -1
        while not Q.empty():
            o = Q.top()
            Q.pop()
            x = o.second.first
            i = o.second.second
            if x == target or target == -1:
                if temp_interval == -1:
                    temp_target = x
                    temp_interval = i
                if min(f[x][i] + target_stay_time, self.MAX_TIME - self.SAFE_EPS) < self.intervals[x][i].second: # TODO: change to max(f[x][i] + target_stay_time, target_last_t) for multi-stage path
                    return x, i
            if vis[x].size() == 0:
                vis[x].resize(self.num_intervals[x], 0)
            if vis[x][i]:
                continue
            vis[x][i] = 1
            self.refresh_nearby_safe_intreval(x, clash_table, clash_node_version)

            # expand by waiting util the next interval
            if i + 1 < self.num_intervals[x]:
                t = self.intervals[x][i + 1].first
                if self.interval_is_safe[x][i + 1]:
                    if t < f[x][i + 1]:
                        f[x][i + 1] = t
                        fa_f[x][i + 1] = ParentNode(0, ipair(x, i))
                        Q.push(pair[double, ipair](-(f[x][i + 1] + heuristic[x]), ipair(x, i + 1)))
                else:
                    if t < g[x][i + 1]:
                        g[x][i + 1] = t
                        fa_g[x][i + 1] = ParentNode(1, ipair(x, i))
            
            # expand by going outwards to another vertex
            # expansion 1: [x, i] -> [x, i + 1]
            # expansion 2: [x, i] -> [y, j] through [e, k], all in the safe interval
            for edge_node in self.inner_edge_out[x]:
                y = edge_node.v_out
                arrival_t_l = f[x][i] + edge_node.cost
                if arrival_t_l >= self.MAX_TIME:
                    continue
                arrival_t_r = min(self.MAX_TIME, self.intervals[x][i].second + edge_node.cost)
                j = self.kth_interval(y, arrival_t_l, clash_table, clash_node_version)
                e = edge_node.e_id
                it = upper_bound(self.edge_safe_intervals[e].begin(), self.edge_safe_intervals[e].end(), dpair(arrival_t_l, self.MAX_TIME))
                if it == self.edge_safe_intervals[e].end():
                    k = self.edge_safe_intervals[e].size() - 1
                else:
                    k = it - self.edge_safe_intervals[e].begin() - 1
                if k < 0:
                    k = 0
                while j < self.num_intervals[y]:
                    if self.intervals[y][j].first >= arrival_t_r:
                        break
                    t = max(self.intervals[y][j].first, arrival_t_l)
                    if t < g[y][j]:
                        g[y][j] = t
                        fa_g[y][j] = ParentNode(1, ipair(x, i))
                    if self.interval_is_safe[y][j]:
                        if k < <int>self.edge_safe_intervals[e].size():
                            while self.edge_safe_intervals[e][k].second < t:
                                k += 1
                        if k < <int>self.edge_safe_intervals[e].size():
                            earliest_safe_t = max(t, self.edge_safe_intervals[e][k].first)
                            if earliest_safe_t < min(arrival_t_r, self.intervals[y][j].second):
                                if earliest_safe_t < f[y][j]:
                                    f[y][j] = earliest_safe_t
                                    fa_f[y][j] = ParentNode(0, ipair(x, i))
                                    Q.push(pair[double, ipair](-(f[y][j] + heuristic[y]), ipair(y, j)))
                    j += 1
        return temp_target, temp_interval

    def plan(self, int start, int target, Graph graph, PathTable path_table, double start_time=0, double target_stay_time=0):
        """Find the path
            start from "start" at time "start_time",
            goto "target" and stay for "target_stay_time",
        with 1. min #collisions; 2. min time
        Output:
            [(e_1, l_1, r_1), ... , (e_n, l_n, r_n), (-1, r_n+target_stay_time, r_n+target_stay_time) or (0, tn, tn)] s.t. e_1.v_in = start, e_n.v_out = target
        """
        if start_time + graph.pairwise_distance[start][target]> self.MAX_TIME:
            return []
        self.max_num_safe_intervals = 0
        self.arrival_time.clear()
        self.fa.clear()
        self.arrival_time.resize(1)
        self.fa.resize(1)
        self.arrival_time[0].resize(self.n + 1)
        self.fa[0].resize(self.n + 1)
        self.refresh_nearby_safe_intreval(start, path_table.clash_table, path_table.clash_node_version)
        cdef int start_interval = self.kth_interval(start, start_time, path_table.clash_table, path_table.clash_node_version)
        cdef int target_interval
        self.arrival_time[0][start][start_interval] = start_time # Warning: we assume the first interval has #collision=0, but actually it can start on an unsafe interval
        cdef dvec dist_to_goal_heuristic
        dist_to_goal_heuristic.resize(self.n + 1)
        cdef int i
        for i in range(1, self.n + 1):
            dist_to_goal_heuristic[i] = graph.pairwise_distance[i][target]
        cdef int collide_limit = 30
        cdef int collide = 0
        while True:
            if self.arrival_time.size() - 1 < <size_t>collide + 1:
                self.arrival_time.resize(collide + 2)
                self.fa.resize(collide + 2)
                self.arrival_time[collide + 1].resize(self.n + 1)
                self.fa[collide + 1].resize(self.n + 1)
            target, target_interval = self.sipp_process(target, target_stay_time, self.arrival_time[collide], self.arrival_time[collide + 1], self.fa[collide], self.fa[collide + 1], dist_to_goal_heuristic, path_table.clash_table, path_table.clash_node_version)
            if target_interval != -1:
                break
            collide += 1
            if collide > max(collide_limit, 3 * self.max_num_safe_intervals):
                print(f"SIPP fails! Too many collisions [= {collide}] have been tried.")
                return []

        final_t = self.arrival_time[collide][target][target_interval]
        if target_stay_time > 0:
            p = [(-1, final_t + target_stay_time, final_t + target_stay_time)]
        else:
            p = [(0, final_t, final_t)]
        cdef int node_x = target
        cdef int node_i = target_interval
        while node_x != start or node_i != start_interval:
            parent_node = self.fa[collide][node_x][node_i]
            parent_x = parent_node.second.first
            parent_i = parent_node.second.second
            if parent_x != node_x:
                for e in graph.edge_in[node_x]:
                    if e.v_in == parent_x:
                        p.append((e.e_id, self.arrival_time[collide][node_x][node_i] - e.cost, self.arrival_time[collide][node_x][node_i]))
                        break
            if parent_node.first:
                collide -= 1
            node_x = parent_x
            node_i = parent_i
        p = p[::-1]
        return p
            
    cdef void update_interval(self, int x, const set[StepNode]& clash_intervals, long long clash_info_version):
        if self.interval_version[x] == clash_info_version:
            return
        self.interval_version[x] = clash_info_version
        self.intervals[x].clear()
        self.interval_is_safe[x].clear()
        self.intervals[x].reserve(2 * clash_intervals.size() + 1)
        self.interval_is_safe[x].reserve(2 * clash_intervals.size() + 1)
        
        cdef double unsafe_l = 0
        cdef double unsafe_r = 0
        cdef double l, r
        for node in clash_intervals:
            l, r = node.t_l, node.t_r
            if unsafe_r >= l - self.SAFE_EPS:
                unsafe_r = max(unsafe_r, r + self.SAFE_EPS)
            else:
                if unsafe_r > 0:
                    self.intervals[x].push_back(dpair(unsafe_l, unsafe_r))
                    self.interval_is_safe[x].push_back(0)
                self.intervals[x].push_back(dpair(unsafe_r, l - self.SAFE_EPS))
                self.interval_is_safe[x].push_back(1)
                unsafe_l, unsafe_r = l - self.SAFE_EPS, r + self.SAFE_EPS
        if unsafe_r > 0:
            if unsafe_r > self.MAX_TIME:
                unsafe_r = self.MAX_TIME
            if unsafe_l < unsafe_r:
                self.intervals[x].push_back(dpair(unsafe_l, unsafe_r))
                self.interval_is_safe[x].push_back(0)
        if unsafe_r < self.MAX_TIME:
            self.intervals[x].push_back(dpair(unsafe_r, self.MAX_TIME))
            self.interval_is_safe[x].push_back(1)
        self.num_intervals[x] = self.intervals[x].size()
            
    cdef void update_edge_safe_interval(self, int e, double e_len, const set[StepNode]& clash_intervals, long long clash_info_version):
        """For edge with length L, the safe interval [t_l, t_r] means safe arrival time that avoid any edge occupy [e, t] (i.e., t_safe in [t_l, t_r] -> t notin [t_safe - L, t_safe]
        """
        if self.edge_safe_interval_version[e] == clash_info_version:
            return
        self.edge_safe_interval_version[e] = clash_info_version
        self.edge_safe_intervals[e].clear()
        self.edge_safe_intervals[e].reserve(clash_intervals.size() + 1)

        cdef double unsafe_r = e_len
        cdef double l, r
        for node in clash_intervals:
            l, r = node.t_l, node.t_r
            if unsafe_r >= l - self.SAFE_EPS:
                unsafe_r = max(unsafe_r, r + e_len + self.SAFE_EPS)
            else:
                self.edge_safe_intervals[e].push_back(dpair(unsafe_r, l - self.SAFE_EPS))
                unsafe_r = r + e_len + self.SAFE_EPS
        if unsafe_r < self.MAX_TIME:
            self.edge_safe_intervals[e].push_back(dpair(unsafe_r, self.MAX_TIME))
