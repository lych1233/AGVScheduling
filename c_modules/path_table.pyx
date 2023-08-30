# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.algorithm cimport sort, binary_search


DEF FLOAT_EPS = 1e-6

cdef class PathTable:
    """Maintain a set of tables with path insert and delete:
        clash_raltion: all [X_id1, X_id2] of clashed 
        clash_table: maintain a set of (agent_id, time_intervals) which makes the vertex (edge) invisible
            clash_node_version: store the timestamp of the last update on that node
            global_clash_table_stamp: global timestamp (number of updates)
        occupy_table: maintain all (agent_id, time_intervals) on the vertex (edge)
        path_buffer: key (int64) -> path, store all the known paths
            path_global_counter: number of paths
            existed_path: all the paths that are inserted (and thus affect clash and occupation)
    """
    def __init__(self, int n, int m):
        self.n, self.m = n, m
        self.clash_relation.resize(self.n + self.m + 1)
        self.occupy_table.resize(self.n + self.m + 1)
        self.clash_table.resize(self.n + self.m + 1)
        self.global_clash_table_stamp = 0
        self.clash_node_version.resize(self.n + self.m + 1, 0)
        self.path_global_counter = 0
        self.existed_path.clear()
        self.path_buffer.clear()
    
    cdef void clear_clash_and_path(self):
        self.global_clash_table_stamp += 1
        cdef int i
        for i in range(1, self.n + self.m + 1):
            self.occupy_table[i].clear()
            if self.clash_table[i].size() > 0:
                self.clash_table[i].clear()
                self.clash_node_version[i] = self.global_clash_table_stamp
        self.existed_path.clear()
    
    def clear_all(self):
        self.clear_clash_and_path()
        self.path_buffer.clear()
    
    def init_clash(self, str node_type, int node_id, int num_v_clash, int[:] v_clash, int num_e_clash, int[:] e_clash):
        cdef int i
        cdef int xid
        if node_type in ["V", "v", "vertex", "Vertex", "vertex_clash"]:
            xid = self.v_to_xid(node_id)
        else:
            xid = self.e_to_xid(node_id)
        self.clash_relation[xid].reserve(num_v_clash + num_e_clash)
        for i in range(num_v_clash):
            self.clash_relation[xid].push_back(self.v_to_xid(v_clash[i]))
        for i in range(num_e_clash):
            self.clash_relation[xid].push_back(self.e_to_xid(e_clash[i]))
        sort(self.clash_relation[xid].begin(), self.clash_relation[xid].end())
    
    cdef void add_clash_interval(self, int xid, const StepNode& step_node, int val):
        if val == 1:
            self.clash_table[xid].insert(step_node)
        else:
            self.clash_table[xid].erase(step_node)
        self.clash_node_version[xid] = self.global_clash_table_stamp
    
    cdef void add_step_node(self, const StepNode& step_node, int val):
        cdef int xid = step_node.loc
        if val == 1:
            self.occupy_table[xid].insert(step_node)
        else:
            self.occupy_table[xid].erase(step_node)
        for yid in self.clash_relation[xid]:
            self.add_clash_interval(yid, step_node, val)
    
    cdef int store_path(self, const vector[StepNode]& path):
        self.path_global_counter += 1
        self.path_buffer[self.path_global_counter] = path
        return self.path_global_counter
    
    cdef vector[StepNode] access_path(self, int path_id):
        cdef vector[StepNode] ans
        cdef StepNode none_node
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            none_node = StepNode(-1, -1, 0, 0)
            ans.push_back(none_node)
        else:
            ans = self.path_buffer[path_id]
        return ans

    cpdef double query_path_length(self, int path_id):
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return -1.0
        path = self.path_buffer[path_id]
        return path[path.size() - 1].t_r

    cpdef double query_path_reach_time(self, int path_id):
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return -1.0
        path = self.path_buffer[path_id]
        if path.size() <= 1:
            return 0.0
        return path[path.size() - 2].t_r
    
    def erase_path(self, int path_id):
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return -1
        self.path_buffer.erase(path_id)
        return 0
    
    def insert_path(self, int path_id):
        """Error Information:
            0: success
            -1: unknown path_id
            1: path already added
        """
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return -1
        if self.existed_path.find(path_id) != self.existed_path.end():
            return 1
        self.existed_path.insert(path_id)
        self.global_clash_table_stamp += 1
        for step_node in self.path_buffer[path_id]:
            self.add_step_node(step_node, 1)
        return 0
    
    def delete_path(self, int path_id):
        """Error Information:
            0: success
            -1: path already deleted (or never been inserted)
        """
        if self.existed_path.find(path_id) == self.existed_path.end():
            return -1
        self.existed_path.erase(path_id)
        self.global_clash_table_stamp += 1
        for step_node in self.path_buffer[path_id]:
            self.add_step_node(step_node, -1)
        return 0
    
    def add_path_to_buffer(self, int agent_id, int start_vertex, list path, Graph graph):
        """transfer python path into c path to avoid precision problem of "double"
            Input:
                start_vertex: should be eqal to e_id.x or -1 for start on an edge
                path [(e_id, t_l, t_r), (e2, l2, r2), ..., ]
                    The last entry is either [(-1, tn+stay_time, tn+stay_time) for stopping on a vertex] or [(0, tn, tn) for stoping on an edge]
            Store: path [StepNode(xid, agent_id, t_l, t_r), StepNode(), ...]
            Return:
                path_id: success store
                -1: format error (e.g., not end with (0, -1) edge)
                -2: not a connected path
                -3: invalid time interval for a edge (t_r - t_l < e.cost)
                -4: time not in order
        """
        if path[-1][0] != -1 and path[-1][0] != 0:
            return -1 # path sould always end with -1 (end on a vertex) or 0 (end on somepoint on an edge)
        cdef vector[StepNode] p
        p.reserve(2 * len(path) + 1)
        cdef int start_e, start_xid
        cdef double start_t_l, start_t_r
        start_e, start_t_l, start_t_r = path[0]
        if start_t_l > FLOAT_EPS:
            start_xid = self.v_to_xid(start_vertex)
            p.push_back(StepNode(start_xid, agent_id, 0.0, start_t_l))
        if start_vertex > 0 and start_e > 0:
            if start_vertex != graph.edges[start_e].v_in:
                print(f"proposed start vertex {start_vertex}, but e_0 starts with {graph.edges[start_e].v_in}")
                return -2
        cdef int v, e
        cdef double t_l, t_r, next_t_l
        for e_in, e_out in zip(path[:-1], path[1:]):
            e, t_l, t_r = e_in
            if t_l > FLOAT_EPS and graph.edges[e].cost > (t_r - t_l) * (1 + FLOAT_EPS) + FLOAT_EPS:
                print(f"error -3: edge {e} length {graph.edges[e].cost} larger then given passing time [{t_l}, {t_r}]")
                return -3
            v = graph.edges[e].v_out
            next_e, next_t_l, next_t_r = e_out
            if next_e > 0:
                if v != graph.edges[next_e].v_in:
                    print(f"disconneted, jump from {v} to {graph.edges[next_e].v_in} with edge {e} to {next_e}")
                    return -2
            if t_r - FLOAT_EPS > next_t_l:
                print(f"time not increasing: {t_r} to {next_t_l} at edge {e}")
                return -4
            p.push_back(StepNode(self.e_to_xid(e), agent_id, t_l, t_r))
            if next_e != 0: # not end on an edge
                p.push_back(StepNode(self.v_to_xid(v), agent_id, t_r, next_t_l))
        return self.store_path(p)
    
    def get_path(self, int path_id):
        """aquire the path with given id
        Output:
            agent_id
            start_vertex (-1 for start on an edge)
            path [(e_id, t_l, t_r), (e2, l2, r2), ..., ]
                The last entry is either [(-1, tn, tn + stay_time) for stopping on a vertex] or [(0, tn, tn) for stoping on an edge]
        """
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return None, None
        path = self.path_buffer[path_id]
        p = []
        if self.xid_is_v(path[0].loc):
            start_vertex = self.xid_to_v(path[0].loc)
        else:
            start_vertex = -1
        for step_node in path:
            if not self.xid_is_v(step_node.loc):
                e = self.xid_to_e(step_node.loc)
                p.append((e, step_node.t_l, step_node.t_r))
        agent_id = path[0].agent_id
        last_node = path[path.size() - 1]
        if self.xid_is_v(last_node.loc):
            p.append((-1, last_node.t_r, last_node.t_r))
        else:
            p.append((0, last_node.t_r, last_node.t_r))
        return agent_id, start_vertex, p
    
    def count_path_pair_collision(self, int path_a, int path_b):
        """Count the number of collisions between two paths
            Input: path_id_1, path_id_2
            Output: #collisions on two paths
                -1: (some) path not exist
                -2: paths correspond to the same agent
        """
        if self.path_buffer.find(path_a) == self.path_buffer.end():
            return -1
        if self.path_buffer.find(path_b) == self.path_buffer.end():
            return -1
        
        p = self.path_buffer[path_a]
        q = self.path_buffer[path_b]
        if p.size() == 0 or q.size() == 0:
            return 0
        if p[0].agent_id == q[0].agent_id:
            return -2
        
        cdef int num_collisions = 0
        cdef size_t i, j
        i = 0
        j = 0
        while i < p.size() and j < q.size():
            if p[i].t_l <= q[j].t_r and q[j].t_l <= p[i].t_r:
                x = p[i].loc
                y = q[j].loc
                if binary_search(self.clash_relation[x].begin(), self.clash_relation[x].end(), y) or binary_search(self.clash_relation[y].begin(), self.clash_relation[y].end(), x):
                    num_collisions += 1
            if p[i].t_r < q[j].t_r:
                i += 1
            else:
                j += 1
        return num_collisions

    cdef pair[vector[StepNode], vector[StepNode]] count_collision_on_path(self, int my_agent_id, int path_id):
        cdef vector[StepNode] my_collisions, other_collisions
        if self.path_buffer.find(path_id) == self.path_buffer.end():
            return pair[vector[StepNode], vector[StepNode]](my_collisions, other_collisions)
        path = self.path_buffer[path_id]
        for step_node in path:
            xid = step_node.loc
            for clash_node in self.clash_table[xid]:
                if clash_node.t_l > step_node.t_r:
                    break
                if my_agent_id == clash_node.agent_id:
                    continue
                if clash_node.t_r >= step_node.t_l:
                    my_collisions.push_back(StepNode(xid, my_agent_id, step_node.t_l, step_node.t_r))
                    other_collisions.push_back(clash_node)
        return pair[vector[StepNode], vector[StepNode]](my_collisions, other_collisions)
