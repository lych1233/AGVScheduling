# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
from cython.operator cimport dereference as deref, preincrement as inc, predecrement as dec
from libcpp.algorithm cimport upper_bound
from libcpp.pair cimport pair
from libcpp.queue cimport queue, priority_queue
from libcpp.set cimport set
from libcpp.vector cimport vector

import time
import numpy as np
cimport numpy as np
np.import_array()

from utils cimport dpair, dvec, ivec, StepNode, RandGen
from graph cimport Graph
from path_table cimport PathTable
from sipp cimport SIPP

DEF inf = 1010000000
DEF t_inf = 1e9

ctypedef pair[int, double] idpair
ctypedef pair[idpair, int] idipair

cdef class LNS:
    cdef int n, m, AGV, mapf_neighbor_size, max_replan_steps
    cdef double max_replan_time
    cdef ivec agv_path_id
    cdef RandGen rng
    cdef Graph graph
    cdef PathTable path_table
    cdef SIPP sipp
    cdef np_rng, CG

    # The following variables are used for neighborhood sampling
    cdef lns_weight
    cdef bint lns_first_init_incomplete
    cdef float max_e_cost
    cdef ivec mapf_start_vertices, mapf_target_vertices, degrees, intersections
    cdef dvec mapf_start_time, mapf_shortest_path
    cdef vector[set[int]] start_occupy_map, target_occupy_map
    cdef set[int] used_delay_agent
    def __init__(self, int n, int m, int num_agvs, Graph graph, PathTable path_table, SIPP sipp):
        self.n, self.m, self.AGV = n, m, num_agvs
        self.mapf_neighbor_size = min(2, int(np.round(2 * (self.AGV ** (1.0 / 3.0)))))
        assert self.mapf_neighbor_size >= 2, f"too few agents ( #AGV = {num_agvs}, neighbor_size = { self.mapf_neighbor_size } )"
        self.graph = graph
        self.path_table = path_table
        self.sipp = sipp
        self.np_rng = np.random.RandomState()

        self.lns_first_init_incomplete = True
        self.max_replan_steps = inf
        self.max_replan_time = t_inf
    
    def set_neighbor_size(self, size_t neighbor_size):
        self.mapf_neighbor_size = min(<size_t>self.AGV, neighbor_size)
    
    def seed(self, int seed):
        self.rng.seed(seed)
        self.np_rng.seed(self.rng.randint(0, (int)(1e9)))
    
    def set_max_replan_steps(self, int max_replan_steps):
        self.max_replan_steps = max_replan_steps
    
    def set_max_replan_time(self, double max_replan_time):
        self.max_replan_time = max_replan_time
    
    cdef initialize_neighbor_sampler(self):
        # Initialize LNS
        cdef double avg_edge_length = 0.0
        cdef int i
        for i in range(1, self.m + 1):
            avg_edge_length += self.graph.edges[i].cost
        avg_edge_length /= self.m
        self.lns_weight = {}
        for select_command in ["collision-based", "target-based", "conflict-random"]:
            self.lns_weight[select_command] = 1
        for select_command in ["agent-based", "map-based", "uniform-random"]:
            self.lns_weight[select_command] = avg_edge_length
        
        # Initialize collision-based
        if self.lns_first_init_incomplete:
            for edge in self.graph.edges:
                self.max_e_cost = max(self.max_e_cost, edge.cost)
        
        # Initialize target-based
        self.start_occupy_map.clear()
        self.start_occupy_map.resize(self.n + self.m + 1)
        self.target_occupy_map.clear()
        self.target_occupy_map.resize(self.n + self.m + 1)
        for i in range(self.AGV):
            xid = self.path_table.v_to_xid(self.mapf_start_vertices[i])
            for yid in self.path_table.clash_relation[xid]:
                self.start_occupy_map[yid].insert(i)
            if self.mapf_target_vertices[i] != -1:
                xid = self.path_table.v_to_xid(self.mapf_target_vertices[i])
                for yid in self.path_table.clash_relation[xid]:
                    self.target_occupy_map[yid].insert(i)
        
        # Initialize agent-based
        self.used_delay_agent.clear()
        
        # Initialize map-based
        if self.lns_first_init_incomplete:
            self.degrees.resize(self.n + 1, 0)
            self.intersections.clear()
            for i in range(1, self.n + 1):
                self.degrees[i] = self.graph.edge_in[i].size() + self.graph.edge_out[i].size()
                if self.degrees[i] >= 3:
                    self.intersections.push_back(i)
        
        self.lns_first_init_incomplete = False
    
    cdef query_next_neighbor_type(self):
        if self.CG.sum() > 0:
            commands = ["collision-based", "target-based", "conflict-random"]
        else:
            commands = ["agent-based", "map-based", "uniform-random"]
        p = np.array([self.lns_weight[command] for command in commands])
        command =  commands[self.np_rng.choice(3, 1, p=p/p.sum())[0]]
        return command
    
    cdef dpair evaluate_solution(self, CG, ivec path_id):
        cdef double total_conflicts = 0.5 * CG.sum()
        cdef double total_length = 0.0
        cdef int p
        for p in path_id:
            total_length += self.path_table.query_path_length(p)
        return dpair(total_conflicts, total_length / self.AGV)
    
    cdef lns_update(self, str select_command, double loss_update_1, double loss_update_2):
        if select_command in ["collision-based", "target-based", "conflict-random"]:
            decay = 0.05
            loss_update = loss_update_1
        else:
            decay = 0.01
            loss_update = loss_update_2 if loss_update_1 >= 0 else 0
        self.lns_weight[select_command] *= 1 - decay
        self.lns_weight[select_command] += decay * max(0, loss_update)
    
    def solveMAPF(self, int[:] start_vertices, int[:] start_edges, int[:] goal_vertices, int[:] start_endpoints, double[:] start_time, double[:] target_stay_time):
        """Input:
            start_vertex, start_edge: (v, -1) for starting on a vertex, (-1, e) for starting on an edge
            goal_vertex: target_vertex for planning
            start_endpoint: v or e.out
            start_time: the time staying on v, or the rest time to pass the edge e; i.e., time to start at start_endpoints
        Output:
            all AGV paths: [[(e_1, l_1, r_1), ...], [path_1], [path_2], ...]
        """
        self.path_table.clear_all()
        self.agv_path_id.resize(self.AGV, -1)

        self.mapf_start_vertices.clear()
        self.mapf_target_vertices.clear()
        self.mapf_start_time.clear()
        self.mapf_shortest_path.clear()
        cdef int i
        for i in range(self.AGV):
            self.mapf_start_vertices.push_back(start_endpoints[i])
            self.mapf_target_vertices.push_back(goal_vertices[i])
            self.mapf_start_time.push_back(start_time[i])
            self.mapf_shortest_path.push_back(start_time[i] + self.graph.pairwise_distance[start_endpoints[i]][goal_vertices[i]])
        self.initialize_neighbor_sampler()
        # CG represents collision graph where CG[x, y] = [if x collides with y?]
        self.CG = np.zeros((self.AGV, self.AGV), dtype=np.intc) 

        for i in range(self.AGV):
            start_path = [(-1, start_time[i], start_time[i])] if start_edges[i] == -1 else [(start_edges[i], 0, start_time[i])]
            self.agv_path_id[i] = self.path_table.add_path_to_buffer(
                agent_id=i,
                start_vertex=start_vertices[i],
                path=start_path,
                graph=self.graph
            )
            self.path_table.insert_path(self.agv_path_id[i])
        cdef int j
        cdef int[:, :] CG_view = self.CG
        for i in range(self.AGV):
            for j in range(self.AGV):
                if i >= j:
                    CG_view[i, j] = CG_view[j, i]
                    continue
                CG_view[i, j] = self.path_table.count_path_pair_collision(self.agv_path_id[i], self.agv_path_id[j]) > 0
        
        cdef int it = 0
        cdef double mapf_start_time = time.time()
        cdef dpair best_loss = dpair(2.0 * self.AGV * self.AGV, 0)
        while it < self.max_replan_steps and time.time() - mapf_start_time < self.max_replan_time:
            # Sample a neighborhood to replan
            if it == 0:
                neighbor = self.get_neighbor("uniform-random", self.AGV)
            else:
                select_command = self.query_next_neighbor_type()
                neighbor = self.get_neighbor(select_command, self.mapf_neighbor_size)
                if neighbor.size() <= 1:
                    it += 1 # avoid infinite loop
                    continue
            
            # Replan using sipp
            new_CG, new_path_id = self.replan_iteration(neighbor, start_vertices, start_edges, goal_vertices, start_endpoints, start_time, target_stay_time)            
            
            # Updarte lns and the best solution
            cur_loss = self.evaluate_solution(new_CG, new_path_id)
            if it > 0:
                self.lns_update(select_command, best_loss.first - cur_loss.first, best_loss.second - cur_loss.second)
            if it == 0 or cur_loss < best_loss:
                best_loss = cur_loss
                self.agv_path_id = new_path_id
                self.CG = new_CG.copy()
            else:
                for i in neighbor:
                    self.path_table.delete_path(new_path_id[i])
                    self.path_table.insert_path(self.agv_path_id[i])
            it += neighbor.size()
        
        return self.CG, [self.path_table.get_path(self.agv_path_id[i])[2] for i in range(self.AGV)]
    
    cdef replan_iteration(self, ivec& neighbor, int[:] start_vertices, int[:] start_edges, int[:] goal_vertices, int[:] start_endpoints, double[:] start_time, double[:] target_stay_time):
        new_path_id = self.agv_path_id
        for i in neighbor:
            self.path_table.delete_path(self.agv_path_id[i])
            new_path_id[i] = -1
        new_CG = self.CG.copy()
        cdef int[:, :] new_CG_view
        new_CG_view = new_CG
        for i in neighbor:
            p = self.sipp.plan(
                start=start_endpoints[i],
                target=goal_vertices[i],
                graph=self.graph,
                path_table=self.path_table,
                start_time=start_time[i],
                target_stay_time=target_stay_time[i],
            )
            if start_edges[i] != -1:
                p = [(start_edges[i], 0.0, start_time[i])] + p                    
            new_path_id[i] = self.path_table.add_path_to_buffer(
                agent_id=i,
                start_vertex=start_vertices[i],
                path=p,
                graph=self.graph
            )
            collision_nodes = self.path_table.count_collision_on_path(i, new_path_id[i]).second
            for j in range(self.AGV):
                if i == j or new_path_id[j] == -1:
                    continue
                new_CG_view[i, j] = new_CG_view[j, i] = 0
            for collision_node in collision_nodes:
                j = collision_node.agent_id
                if i == j or new_path_id[j] == -1:
                    continue
                new_CG_view[i, j] = new_CG_view[j, i] = 1
            self.path_table.insert_path(new_path_id[i])                     
        return new_CG, new_path_id      
    
    cdef ivec get_neighbor(self, str select_command, size_t neighbor_size):
        cdef ivec neighbor
        if select_command == "collision-based":
            neighbor = self.generate_collision_based_neighbor(neighbor_size)
        elif select_command == "target-based":
            neighbor = self.generate_target_based_neighbor(neighbor_size)
        elif select_command == "conflict-random":
            neighbor = self.generate_conflict_random_neighbor(neighbor_size)
        elif select_command == "agent-based":
            neighbor = self.generate_agent_based_neighbor(neighbor_size)
        elif select_command == "map-based":
            neighbor = self.generate_map_based_neighbor(neighbor_size)
        elif select_command == "uniform-random":
            neighbor = self.generate_uniform_random_neighbor(neighbor_size)
        else:
            raise NotImplementedError
        
        cdef int[:] p = self.np_rng.permutation(neighbor.size()).astype(np.intc)
        cdef ivec ans
        ans.resize(neighbor.size())
        cdef size_t i
        for i in range(neighbor.size()):
            ans[i] = neighbor[p[i]]
        return ans
    
    cdef ivec np_to_vec(self, int sz, int[:] neighbor):
        cdef ivec ans
        ans.reserve(sz)
        cdef int i
        for i in range(sz):
            ans.push_back(neighbor[i])
        return ans
    
    # The following part is for collision-based sampling (for conflict repair)
    cdef void get_nodes_on_xid_t(self, int loc, float t_l, float t_r, float interval_max, float t_inf, vector[StepNode]& node_list):
        node_set = self.path_table.clash_table[loc]
        node_list.clear()
        if node_set.size() == 0:
            return
        it = node_set.upper_bound(StepNode(inf, inf, t_r, t_inf))
        while it != node_set.begin():
            dec(it)
            node = deref(it)
            if node.t_l < t_r and t_l < node.t_r:
                node_list.push_back(node)
            if node.t_r + interval_max < t_l:
                break
    
    cdef int random_walk(self, int agv, float interval_max, float t_max):
        cdef double t = self.path_table.query_path_length(self.agv_path_id[agv])
        cdef double t_l = self.rng.uniform(0, t)
        path = self.path_table.access_path(self.agv_path_id[agv])
        it = upper_bound(path.begin(), path.end(), StepNode(inf, inf, t_l, t_inf))
        cdef int xid
        cdef double t_r
        if it == path.end():
            xid = path.back().loc
            t_r = path.back().t_r
        else:
            xid = path[it - path.begin() - 1].loc
            t_r = path[it - path.begin() - 1].t_r
        cdef vector[StepNode] encounter_nodes, next_nodes
        cdef float wait_time
        while t_l < t_max:
            self.get_nodes_on_xid_t(xid, t_l, t_r, interval_max, t_inf, encounter_nodes)
            if encounter_nodes.size() > 1 or encounter_nodes.size() > 0 and encounter_nodes[0].agent_id != agv:
                break
            if self.path_table.xid_is_v(xid):
                v = self.path_table.xid_to_v(xid)
                next_nodes.clear()
                wait_time = t_inf
                for edge in self.graph.edge_out[v]:
                    next_nodes.push_back(StepNode(self.path_table.e_to_xid(edge.e_id), agv, t_r, t_r + edge.cost))
                    wait_time = min(wait_time, edge.cost)
                next_nodes.push_back(StepNode(xid, agv, t_r, t_r + wait_time))
            else:
                e = self.path_table.xid_to_e(xid)
                v_out = self.graph.edges[e].v_out
                wait_time = self.graph.edges[e].cost
                next_nodes.clear()
                next_nodes.push_back(StepNode(self.path_table.v_to_xid(v_out), agv, t_r, t_r))
                next_nodes.push_back(StepNode(xid, agv, t_r, t_r + wait_time))
            next_node = next_nodes[self.rng.randint(0, next_nodes.size() - 1)]
            xid, t_l, t_r = next_node.loc, next_node.t_l, next_node.t_r
        if t >= t_max or encounter_nodes.size() == 0:
            return -1
        return encounter_nodes[self.rng.randint(0, encounter_nodes.size() - 1)].agent_id
    
    cdef ivec generate_collision_based_neighbor(self, size_t neighbor_size):
        cdef int[:, :] CG_view = self.CG
        agv_collided = np.any(self.CG, 1)
        cdef int agv = self.np_rng.choice(self.AGV, 1, p=agv_collided/agv_collided.sum())[0]

        cdef ivec CC # get the connected component on the collision graph
        cdef set[int] CC_set
        CC.push_back(agv)
        CC_set.insert(agv)
        cdef size_t q_head = 0
        cdef int i
        while q_head < CC.size():
            x = CC[q_head]
            q_head += 1
            for i in range(self.AGV):
                if CG_view[x, i] > 0 and CC_set.find(i) == CC_set.end():
                    CC.push_back(i)
                    CC_set.insert(i)
        
        cdef ivec ans
        cdef set[int] ans_set
        cdef int failed_attempts, total_attempts
        cdef int max_failed_attempts = 10
        cdef int max_total_attempts = neighbor_size * 30
        cdef double interval_max = 10 * self.max_e_cost
        if CC.size() <= neighbor_size:
            for x in CC:
                ans.push_back(x)
                ans_set.insert(x)
            t_max = 0.0
            for i in range(self.AGV):
                t_max = max(t_max, self.path_table.query_path_length(self.agv_path_id[i]))
            failed_attempts, total_attempts = 0, 0
            while failed_attempts < max_failed_attempts and total_attempts < max_total_attempts and ans.size() < neighbor_size:
                x = ans[self.rng.randint(0, ans.size() - 1)]
                y = self.random_walk(x, interval_max, t_max)
                if y != -1:
                    if ans_set.find(y) == ans_set.end():
                        ans.push_back(y)
                        ans_set.insert(y)
                else:
                    failed_attempts += 1
                total_attempts += 1
        else:
            i = agv
            ans.push_back(i)
            ans_set.insert(i)
            total_attempts = 0
            while total_attempts < max_total_attempts and ans.size() < neighbor_size:
                i = self.np_rng.choice(self.AGV, 1, p=self.CG[i]/self.CG[i].sum())[0]
                if ans_set.find(i) == ans_set.end():
                    ans.push_back(i)
                    ans_set.insert(i)
                total_attempts += 1
        return ans
    
    # The following part is for target-based sampling (for conflict repair)
    cdef set[int] findpath_min_target_collision_set(self, int agv):
        cdef int s = self.mapf_start_vertices[agv], goal = self.mapf_target_vertices[agv]
        if goal == -1:
            return set[int]()
        cdef priority_queue[idipair] Q
        cdef vector[idpair] d
        d.resize(self.n + 1, idpair(inf, t_inf))
        d[s] = idpair(self.target_occupy_map[self.path_table.v_to_xid(s)].size(), 0.0)
        Q.push(idipair(d[s], s))
        cdef vector[bint] vis = vector[bint](self.n + 1, 0)
        cdef ivec fa = ivec(self.n + 1, -1)
        cdef set[int] A_target
        while not Q.empty():
            x = Q.top().second
            Q.pop()
            if x == goal:
                while x != -1:
                    for agv_on_target in self.target_occupy_map[self.path_table.v_to_xid(x)]:
                        A_target.insert(agv_on_target)
                    for e_in in self.graph.edge_in[x]:
                        if e_in.v_in == fa[x]:
                            for agv_on_target in self.target_occupy_map[self.path_table.e_to_xid(e_in.e_id)]:
                                A_target.insert(agv_on_target)
                            break
                    x = fa[x]
                break
            if vis[x]:
                continue
            vis[x] = 1
            for o in self.graph.edge_out[x]:
                y = o.v_out
                new_dy = d[x]
                new_dy.first += self.target_occupy_map[self.path_table.v_to_xid(y)].size()
                new_dy.first += self.target_occupy_map[self.path_table.e_to_xid(o.e_id)].size()
                new_dy.second += o.cost
                if new_dy < d[y]:
                    fa[y] = x
                    d[y] = new_dy
                    Q.push(idipair(idpair(-new_dy.first, -new_dy.second), y))
        return A_target
    
    cdef ivec generate_target_based_neighbor(self, size_t neighbor_size):
        num_collisions = self.CG.sum(-1)
        cdef int agv = self.np_rng.choice(self.AGV, 1, p=num_collisions/num_collisions.sum())[0]
        cdef ivec ans
        cdef set[int] ans_set
        ans.push_back(agv)
        ans_set.insert(agv)

        cdef ivec A_start
        cdef set[int] A_start_set
        path = self.path_table.access_path(self.agv_path_id[agv])
        for node in path:
            for agent in self.start_occupy_map[node.loc]:
                if agent == agv or A_start_set.find(agent) != A_start_set.end():
                    continue
                A_start.push_back(agent)
        A_target = self.findpath_min_target_collision_set(agv)
        if A_start.size() + A_target.size() == 0:
            return ans
        
        cdef ivec A_target_list, A_target_shuffle
        A_target_list.reserve(A_target.size())
        A_target_shuffle.reserve(A_target.size())
        for agent in A_target:
            A_target_list.push_back(agent)
        rand_perm = self.np_rng.permutation(A_target.size())
        for i in range(A_target.size()):
            A_target_shuffle.push_back(A_target_list[rand_perm[i]])

        if A_start.size() > 0:
            ans.push_back(A_start[0])
            ans_set.insert(A_start[0])
            if ans.size() == neighbor_size:
                return ans
        for agent in A_target_shuffle:
            if ans_set.find(agent) == ans_set.end():
                ans.push_back(agent)
                ans_set.insert(agent)
                if ans.size() == neighbor_size:
                    return ans
        for agent in A_start:
            if ans_set.find(agent) == ans_set.end():
                ans.push_back(agent)
                ans_set.insert(agent)
                if ans.size() == neighbor_size:
                    return ans
        
        cdef set[int] used_agents
        cdef int key_agv, rand_kth
        while used_agents.size() < ans.size():
            key_agv = ans[self.rng.randint(0, ans.size() - 1)]
            used_agents.insert(key_agv)
            path = self.path_table.access_path(self.agv_path_id[key_agv])
            A_target.clear()
            for node in path:
                for agent in self.target_occupy_map[node.loc]:
                    A_target.insert(agent)
            if A_target.size() > 0:
                rand_kth = self.rng.randint(0, A_target.size() - 1)
                for new_agv in A_target:
                    if rand_kth == 0:
                        break
                    rand_kth -= 1
                if ans_set.find(new_agv) == ans_set.end():
                    ans.push_back(new_agv)
                    ans_set.insert(new_agv)
        return ans

    # The following part is for random sampling (for conflict repair)
    cdef ivec generate_conflict_random_neighbor(self, size_t neighbor_size):
        conflict_deg = self.CG.sum(1)
        prob = (conflict_deg + 1) / ((conflict_deg + 1).sum())
        neighbor = self.np_rng.choice(self.AGV, neighbor_size, replace=False, p=prob).astype(np.intc)
        return self.np_to_vec(len(neighbor), neighbor)
    
    # The following part is for agent-based sampling (for solution refine)
    cdef bint update_ans_with_clash(self, int xid, int agv, float t_l, float t_r, ivec& ans, set[int]& ans_set, size_t neighbor_size):
        for node in self.path_table.clash_table[xid]:
            if node.t_l < t_r and t_l < node.t_r:
                a = node.agent_id
                if a != -1 and a != agv and ans_set.find(a) == ans_set.end():
                    ans.push_back(a)
                    ans_set.insert(a)
                    if ans.size() == neighbor_size:
                        return True
        return False
    
    cdef void optimize_walk(self, int agv, float t, ivec& ans, set[int]& ans_set, size_t neighbor_size):
        cdef int goal = self.mapf_target_vertices[agv]
        if goal == -1:
            return
        path = self.path_table.access_path(self.agv_path_id[agv])
        cdef double t_final = self.path_table.query_path_reach_time(self.agv_path_id[agv])
        it = upper_bound(path.begin(), path.end(), StepNode(inf, inf, t, t_inf))
        cdef int xid, v
        if it == path.end():
            xid = path.back().loc
            t = path.back().t_l
        else:
            xid = path[it - path.begin() - 1].loc
            t = path[it - path.begin() - 1].t_l
        if self.path_table.xid_is_v(xid):
            v = self.path_table.xid_to_v(xid)
        else:
            v = self.graph.edges[self.path_table.xid_to_e(xid)].v_in
        
        cdef vector[idpair] next_nodes
        cdef int k, new_v
        cdef float wait_time, new_t
        while t < t_final:
            next_nodes.clear()
            wait_time = t_inf
            for edge in self.graph.edge_out[v]:
                next_nodes.push_back(idpair(edge.v_out, t + edge.cost))
                wait_time = min(wait_time, edge.cost)
            next_nodes.push_back(idpair(v, t + wait_time))
            while not next_nodes.empty():
                k = self.rng.randint(0, next_nodes.size() - 1)
                new_v = next_nodes[k].first
                new_t = next_nodes[k].second
                if new_t + self.graph.pairwise_distance[new_v][goal] < t_final:
                    if v == new_v:
                        if self.update_ans_with_clash(self.path_table.v_to_xid(v), agv, t, new_t, ans, ans_set, neighbor_size):
                            return
                    else:
                        if self.update_ans_with_clash(self.path_table.v_to_xid(v), agv, t, t, ans, ans_set, neighbor_size):
                            return
                        for edge in self.graph.edge_out[v]:
                            if edge.v_out == new_v:
                                if self.update_ans_with_clash(self.path_table.e_to_xid(edge.e_id), agv, t, new_t, ans, ans_set, neighbor_size):
                                    return
                                break
                    break
                next_nodes.erase(next_nodes.begin() + k)
            v = new_v
            t = new_t
            if next_nodes.empty():
                break
        self.update_ans_with_clash(self.path_table.v_to_xid(v), agv, t, t, ans, ans_set, neighbor_size)
        
    cdef ivec generate_agent_based_neighbor(self, size_t neighbor_size):
        cdef float max_delay = 0.0
        cdef int agv = -1
        cdef int i
        for i in range(self.AGV):
            if self.used_delay_agent.find(i) != self.used_delay_agent.end():
                continue
            delay = self.path_table.query_path_reach_time(self.agv_path_id[i]) - self.mapf_shortest_path[i]
            if delay > max_delay:
                max_delay = delay
                agv = i

        if agv == -1 or max_delay < self.sipp.SAFE_EPS:
            self.used_delay_agent.clear()
            return vector[int]()
        self.used_delay_agent.insert(agv)
        if self.used_delay_agent.size() == <size_t>self.AGV:
            self.used_delay_agent.clear()
        
        cdef ivec ans
        cdef set[int] ans_set
        ans.push_back(agv)
        ans_set.insert(agv)
        self.optimize_walk(agv, self.mapf_start_time[agv], ans, ans_set, neighbor_size)
        if ans.size() == neighbor_size:
            return ans
        for i in range(10):
            t = self.path_table.query_path_reach_time(self.agv_path_id[agv])
            t = self.rng.uniform(self.mapf_start_time[agv], t)
            self.optimize_walk(agv, t, ans, ans_set, neighbor_size)
            if ans.size() == neighbor_size:
                return ans
            agv = ans[self.rng.randint(0, ans.size() - 1)]
        return ans
    
    # The following part is for map-based sampling (for solution refine)
    cdef bint collect_agents_on_vertex(self, int x, ivec& ans, set[int]& ans_set, size_t neighbor_size):
        occ_table = self.path_table.occupy_table[x]
        if occ_table.size() == 0:
            return False
        t = deref(occ_table.rbegin()).t_r
        t = self.rng.uniform(0, t)
        it_l = occ_table.upper_bound(StepNode(inf, inf, t, t_inf))
        it_r = occ_table.upper_bound(StepNode(inf, inf, t, t_inf))
        while it_l != occ_table.begin() or it_r != occ_table.end():
            if it_l != occ_table.begin():
                dec(it_l)
                a = deref(it_l).agent_id
                if a != -1 and ans_set.find(a) == ans_set.end():
                    ans.push_back(a)
                    ans_set.insert(a)
                    if ans.size() == neighbor_size:
                        return True
            if it_r != occ_table.end():
                a = deref(it_r).agent_id
                if a != -1 and ans_set.find(a) == ans_set.end():
                    ans.push_back(a)
                    ans_set.insert(a)
                    if ans.size() == neighbor_size:
                        return True
                inc(it_r)
            if it_r != occ_table.end():
                node1 = deref(it_l)
                node2 = deref(it_r)
        return False

    cdef ivec generate_map_based_neighbor(self, size_t neighbor_size):
        cdef int start_v = self.intersections[self.rng.randint(0, self.intersections.size() - 1)]
        cdef queue[int] Q
        cdef set[int] used_v
        Q.push(start_v)
        used_v.insert(start_v)
        cdef ivec ans
        cdef set[int] ans_set
        while not Q.empty():
            x = Q.front()
            Q.pop()
            if self.degrees[x] >= 3:
                if self.collect_agents_on_vertex(x, ans, ans_set, neighbor_size):
                    return ans
            for e_in in self.graph.edge_in[x]:
                y = e_in.v_in
                if used_v.find(y) == used_v.end():
                    used_v.insert(y)
                    Q.push(y)
            for e_out in self.graph.edge_out[x]:
                y = e_out.v_out
                if used_v.find(y) == used_v.end():
                    used_v.insert(y)
                    Q.push(y)
        return ans

    # The following part is for random sampling (for solution refine)
    cdef ivec generate_uniform_random_neighbor(self, size_t neighbor_size):
        neighbor = self.np_rng.choice(self.AGV, neighbor_size, replace=False).astype(np.intc)
        return self.np_to_vec(len(neighbor), neighbor)
