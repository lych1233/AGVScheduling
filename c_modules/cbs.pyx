# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
from cython.operator cimport dereference as deref
from libcpp.memory cimport shared_ptr, make_shared
from libcpp.unordered_map cimport unordered_map
from libcpp.pair cimport pair
from libcpp.queue cimport priority_queue
from libcpp.set cimport set
from libcpp.vector cimport vector

import time
import numpy as np
cimport numpy as np
np.import_array()

from utils cimport ipair, dvec, ivec, RandGen, CBSConstraint, CBSConflict, CBSNode
from graph cimport Graph
from path_table cimport PathTable
from sipp cimport SIPP


DEF inf = 1010000000
DEF t_inf = 1e9

ctypedef pair[double, int] dipair
ctypedef pair[dipair, int] diipair


cdef class CBS:
    cdef int n, m, AGV, max_expand_node, cbs_node_counter
    cdef double max_search_time
    cdef RandGen rng
    cdef Graph graph
    cdef PathTable path_table
    cdef SIPP sipp
    cdef ivec start_vertices, start_edges, goal_vertices, start_endpoints
    cdef dvec start_time, target_stay_time
    
    cdef CBSNode* best_solution
    cdef vector[CBSConstraint] initial_occupancies
    cdef unordered_map[int, CBSNode*] cbs_node_buffer
    cdef priority_queue[diipair] cbs_tree
    def __init__(self, int n, int m, int num_agvs, Graph graph, PathTable path_table, SIPP sipp):
        self.n, self.m, self.AGV = n, m, num_agvs
        self.graph = graph
        self.path_table = path_table
        self.sipp = sipp
        self.max_expand_node = inf
        self.max_search_time = t_inf
        
    def set_max_expand_node(self, int max_expand_node):
        self.max_expand_node = max_expand_node
    
    def set_max_search_time(self, double max_search_time):
        self.max_search_time = max_search_time
    
    def seed(self, int seed):
        self.rng.seed(seed)
    
    cdef add_initial_occupancies(self):
        self.initial_occupancies.clear()
        self.initial_occupancies.reserve(self.AGV)
        cdef int i, xid
        for i in range(self.AGV):
            if self.start_vertices[i] > 0:
                xid = self.path_table.v_to_xid(self.start_vertices[i])
            else:
                xid = self.path_table.e_to_xid(self.start_edges[i])
            self.initial_occupancies.push_back(CBSConstraint(xid, i, 0.0, self.start_time[i], -1))
    
    cdef void add_constraint(self, CBSConstraint& constraint, int val):
        self.path_table.add_clash_interval(constraint.loc, constraint.forbid(), val)
    
    cdef void add_occupancy(self, CBSConstraint& occupancy, int val):
        self.path_table.add_step_node(occupancy.occupy(), val)
    
    cdef int plan_path(self, int agent_id):  
        p = self.sipp.plan(
            start=self.start_endpoints[agent_id],
            target=self.goal_vertices[agent_id],
            graph=self.graph,
            path_table=self.path_table,
            start_time=self.start_time[agent_id],
            target_stay_time=self.target_stay_time[agent_id],
        )
        if self.start_edges[agent_id] != -1:
            p = [(self.start_edges[agent_id], 0.0, self.start_time[agent_id])] + p
        path_id = self.path_table.add_path_to_buffer(
            agent_id=agent_id,
            start_vertex=self.start_vertices[agent_id],
            path=p,
            graph=self.graph
        )
        return path_id
    
    cdef ivec get_node_paths(self, CBSNode* node):
        cdef ivec paths
        paths.resize(self.AGV, -1)
        while node != NULL:
            for agent_path in node.agent_paths:
                agent_id = agent_path.first
                path_id = agent_path.second
                if paths[agent_id] == -1:
                    paths[agent_id] = path_id
            node = node.fa
        return paths
    
    cdef double query_path_length(self, int path_id):
        if path_id == -1:
            return 0.0
        return self.path_table.query_path_length(path_id)
    
    cdef void post_summary(self, CBSNode* node):
        cdef ivec paths = self.get_node_paths(node.fa)
        
        cdef set[int] new_agents
        if node.fa != NULL:
            node.path_length_sum = node.fa.path_length_sum
        else:
            node.path_length_sum = 0
        for agent_path in node.agent_paths:
            agent = agent_path.first
            node.path_length_sum -= self.query_path_length(paths[agent])
            paths[agent] = agent_path.second
            new_agents.insert(agent)
            node.path_length_sum += self.query_path_length(paths[agent])
        
        if node.fa != NULL:
            node.conflicts.reserve(node.fa.conflicts.size())
            for conflict in node.fa.conflicts:
                if new_agents.find(deref(conflict).agent_id_0) != new_agents.end():
                    continue
                if new_agents.find(deref(conflict).agent_id_1) != new_agents.end():
                    continue
                node.conflicts.push_back(conflict)

        self.path_table.clear_clash_and_path()
        cdef int i, j, num_collisions, agent_0, agent_1, xid_0, xid_1
        cdef double t, e_cost_0, e_cost_1
        cdef shared_ptr[CBSConflict] new_conflict
        for i in range(self.AGV):
            if new_agents.find(i) == new_agents.end():
                self.path_table.insert_path(paths[i])
        for i in new_agents:
            collision_nodes = self.path_table.count_collision_on_path(i, paths[i])
            num_collisions = collision_nodes.first.size()
            for j in range(num_collisions):
                agent_0 = (collision_nodes.first)[j].agent_id
                agent_1 = (collision_nodes.second)[j].agent_id
                xid_0 = (collision_nodes.first)[j].loc
                xid_1 = (collision_nodes.second)[j].loc
                if self.path_table.xid_is_v(xid_0) or self.path_table.xid_is_v(xid_1):
                    continue # TODO: add [V, E] conflict
                e_cost_0 = self.graph.edges[self.path_table.xid_to_e(xid_0)].cost
                e_cost_1 = self.graph.edges[self.path_table.xid_to_e(xid_1)].cost
                t = max((collision_nodes.first)[j].t_l, (collision_nodes.second)[j].t_l)
                new_conflict = make_shared[CBSConflict]()
                deref(new_conflict).edge_to_edge_conflict(agent_0, agent_1, xid_0, xid_1, t, e_cost_0, e_cost_1)
                node.conflicts.push_back(new_conflict)
            # TODO: classify conflict
            self.path_table.insert_path(paths[i])
        node.conflicted_pairs = node.conflicts.size()
    
    cdef void update_solution(self, CBSNode* node):
        if node.conflicted_pairs < self.best_solution.conflicted_pairs:
            self.best_solution = node
    
    cdef void push_node(self, CBSNode* node):
        self.update_solution(node)
        
        self.cbs_node_counter += 1
        self.cbs_node_buffer[self.cbs_node_counter] = node
        self.cbs_tree.push(diipair(dipair(-node.path_length_sum, -node.conflicted_pairs), self.cbs_node_counter))
    
    cdef CBSNode* get_root_node(self):
        cdef CBSNode* root = new CBSNode()
        root.agent_paths.resize(self.AGV)
        self.path_table.clear_clash_and_path()
        for occupancy in self.initial_occupancies:
            self.add_occupancy(occupancy, 1)
        cdef int i
        for i in range(self.AGV):
            for occupancy in self.initial_occupancies:
                if occupancy.agent_id == i:
                    self.add_occupancy(occupancy, -1)
            root.agent_paths[i] = ipair(i, self.plan_path(i))
            for occupancy in self.initial_occupancies:
                if occupancy.agent_id == i:
                    self.add_occupancy(occupancy, 1) 
        self.post_summary(root)
        return root
    
    cdef bint verify_solution(self, CBSNode* node):
        return node.conflicted_pairs == 0
    
    cdef shared_ptr[CBSConflict] choose_conflict(self, CBSNode* node):
        cdef shared_ptr[CBSConflict] best = node.conflicts[0]
        for conflict in node.conflicts:
            if deref(best) < deref(conflict):
                best = conflict
        return best
    
    cdef void traverse_add_constraints(self, CBSNode* node, int agent):
        while node != NULL:
            for constraint in node.last_constraints:
                if constraint.agent_id == agent:
                    self.add_constraint(constraint, 1)
            node = node.fa
    
    cdef bint build_children(self, CBSNode* node):
        cdef set[int] new_agents
        for constraint in node.last_constraints:
            new_agents.insert(constraint.agent_id)
        for agent in new_agents:
            self.path_table.clear_clash_and_path()
            for occupancy in self.initial_occupancies:
                if occupancy.agent_id != agent:
                    self.add_occupancy(occupancy, 1)
            self.traverse_add_constraints(node, agent)
            path = self.plan_path(agent)
            node.agent_paths.push_back(ipair(agent, path))
            if self.path_table.count_collision_on_path(agent, path).first.size() > 0:
                return False
        self.post_summary(node)
        return True
    
    # cdef print_conflict(self, CBSConflict conflict):
    #     print("agent_id_0, agent_id_1", conflict.agent_id_0, conflict.agent_id_1)
    #     for constraint in conflict.constraints_0:
    #         print("constraint_0:", "constraint.loc, constraint.agent_id, constraint.t_l, constraint.t_r", constraint.loc, constraint.agent_id, constraint.t_l, constraint.t_r)
    #     print() # TODO remove
    
    cdef void expand(self, CBSNode* cur):
        cdef shared_ptr[CBSConflict] conflict = self.choose_conflict(cur)        
        cdef vector[CBSNode*] children
        cdef int i
        for i in range(2):
            children.push_back(new CBSNode())
            children[i].fa = cur
            children[i].last_constraints = deref(conflict).get_constraints(i)
            if self.build_children(children[i]):
                self.push_node(children[i])
            else:
                del children[i]

    cdef bint cbs_solve(self):
        self.cbs_node_counter = 0
        for cbs_node in self.cbs_node_buffer:
            del cbs_node.second
        self.cbs_node_buffer.clear()
        while not self.cbs_tree.empty():
            self.cbs_tree.pop()
        self.add_initial_occupancies()
        
        cdef CBSNode* root = self.get_root_node()
        self.best_solution = root
        self.push_node(root)
        cdef CBSNode* cur
        num_expand_nodes = 0
        search_start_time = time.time()
        while not self.cbs_tree.empty():
            num_expand_nodes += 1
            if num_expand_nodes > self.max_expand_node:
                break
            if time.time() - search_start_time > self.max_search_time:
                break
            
            cur = self.cbs_node_buffer[self.cbs_tree.top().second]
            if self.verify_solution(cur):
                self.best_solution = cur
                return True
            self.expand(cur)
            cur.clear()
            self.cbs_tree.pop()
        print("cbs fails...")
        print("total nodes", num_expand_nodes)
        print("total time", time.time() - search_start_time)
        return False
        
    
    def solveMAPF(self, int[:] start_vertices, int[:] start_edges, int[:] goal_vertices, int[:] start_endpoints, double[:] start_time, double[:] target_stay_time):
        """Input:
            start_vertex, start_edge: (v, -1) for starting on a vertex, (-1, e) for starting on an edge
            goal_vertex: target_vertex for planning
            start_endpoint: v or e.out
            start_time: the time staying on v, or the rest time to pass the edge e; i.e., time to start at start_endpoints
        Output:
            all AGV paths: [[(e_1, l_1, r_1), ...], [path_1], [path_2], ...]
        """
        cdef int i
        self.start_vertices.resize(self.AGV)
        self.start_edges.resize(self.AGV)
        self.goal_vertices.resize(self.AGV)
        self.start_endpoints.resize(self.AGV)
        self.start_time.resize(self.AGV)
        self.target_stay_time.resize(self.AGV)
        for i in range(self.AGV):
            self.start_vertices[i] = start_vertices[i]
            self.start_edges[i] = start_edges[i]
            self.goal_vertices[i] = goal_vertices[i]
            self.start_endpoints[i] = start_endpoints[i]
            self.start_time[i] = start_time[i]
            self.target_stay_time[i] = target_stay_time[i]
        
        self.path_table.clear_all()
        if self.cbs_solve():
            print("cbs finishes!!!")
        else:
            print("cbs fails......")
        cdef CBSNode ans = deref(self.best_solution)
        
        cdef ivec paths = self.get_node_paths(&ans)
        
        self.path_table.clear_clash_and_path()
        CG = np.zeros((self.AGV, self.AGV), dtype=np.intc)
        cdef int [:, :] CG_view = CG
        for i in range(self.AGV):
            collision_nodes = self.path_table.count_collision_on_path(i, paths[i])
            for other_collision in collision_nodes.second:
                j = other_collision.agent_id
                CG_view[i, j] = CG_view[j, i] = 1
            # TODO: classify conflict
            self.path_table.insert_path(paths[i])
        return CG, [self.path_table.get_path(paths[i])[2] for i in range(self.AGV)]
