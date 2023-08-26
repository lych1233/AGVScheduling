# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.pair cimport pair
from libcpp.queue cimport queue, priority_queue

DEF inf = 1e30
ctypedef pair[double, int] dipair


cdef class Graph:
    """The graph contains basic vertex and edge information:
        - n, m: number of vertices, edges
        - e_in, e_out: e_in[x] -> vector< edge(e_id, v_in=x, v_out=y, cost=e_length) >
    There are some tools to use:
        - compute_pairwise_distance (O(NMlogM)): pairwise shortest path computed by n dijkstra calls
    """
    def __init__(self, int n, int m, int[:] x, int[:] y, double[:] z):
        self.n, self.m = n, m
        self.edge_in.clear()
        self.edge_out.clear()
        self.edges.clear()
        cdef int i
        self.edge_in.resize(self.n + 1)
        self.edge_out.resize(self.n + 1)
        for i in range(1, m + 1):
            self.edge_out[x[i]].push_back(EdgeNode(i, x[i], y[i], z[i]))
            self.edge_in[y[i]].push_back(EdgeNode(i, x[i], y[i], z[i]))
        self.edges.resize(self.m + 1)
        for i in range(1, m + 1):
            self.edges[i] = EdgeNode(i, x[i], y[i], z[i])
    
    cdef void single_source_dijkstra(self, int s, dvec& d, ivec& depth, ivec &fa):
        cdef priority_queue[dipair] Q
        d[s] = 0
        depth[s] = 1
        Q.push(dipair(0, s))
        cdef vector[bint] vis = vector[bint](self.n + 1, 0)
        while not Q.empty():
            x = Q.top().second
            Q.pop()
            if vis[x]:
                continue
            vis[x] = 1
            for o in self.edge_out[x]:
                y, z = o.v_out, o.cost
                if d[x] + z < d[y]:
                    d[y] = d[x] + z
                    Q.push(dipair(-d[y], y))
                    depth[y] = depth[x] + 1
                    fa[y] = x

    cpdef graph_query_nearby_vertices(self, int neighbor_size, int[:, :] query_v, int[:, :] query_depth, int[:, :] query_fa):
        cdef queue[int] neighbor
        cdef priority_queue[dipair] Q
        cdef dvec d = dvec(self.n + 1, inf)
        cdef ivec depth = ivec(self.n + 1, 0)
        cdef ivec fa = ivec(self.n + 1, 0)
        cdef vector[bint] vis = vector[bint](self.n + 1, 0)
        cdef int start_vertex, neighbor_counter
        for start_vertex in range(1, self.n + 1):
            while not Q.empty():
                x = Q.top().second
                d[x] = inf
                Q.pop()
            d[start_vertex] = 0
            depth[start_vertex] = 1
            fa[start_vertex] = 0
            Q.push(dipair(0, start_vertex))
            neighbor_counter = 0
            while not Q.empty():
                x = Q.top().second
                Q.pop()
                if vis[x]:
                    continue
                vis[x] = 1
                neighbor.push(x)
                neighbor_counter += 1
                if neighbor_counter == neighbor_size:
                    break
                for o in self.edge_out[x]:
                    y, z = o.v_out, o.cost
                    if d[x] + z < d[y]:
                        fa[y] = x
                        depth[y] = depth[x] + 1
                        d[y] = d[x] + z
                        Q.push(dipair(-d[y], y))
            neighbor_counter = 0
            while not neighbor.empty():
                x = neighbor.front()
                neighbor.pop()
                query_v[start_vertex, neighbor_counter] = x
                query_depth[start_vertex, neighbor_counter] = depth[x]
                query_fa[start_vertex, neighbor_counter] = fa[x]
                d[x] = inf
                vis[x] = 0
                neighbor_counter += 1

    def compute_pairwise_distance(self):
        self.pairwise_distance.clear()
        self.pairwise_distance.resize(self.n + 1, dvec(self.n + 1, inf))
        self.shortest_graph_depth.clear()
        self.shortest_graph_depth.resize(self.n + 1, ivec(self.n + 1, 0))
        self.shortest_graph_fa.clear()
        self.shortest_graph_fa.resize(self.n + 1, ivec(self.n + 1, 0))
        cdef int i
        for i in range(1, self.n + 1):
            self.single_source_dijkstra(i, self.pairwise_distance[i], self.shortest_graph_depth[i], self.shortest_graph_fa[i])
    
    def clone_pairwise_distance(self, double[:, :] d):
        cdef int i, j
        for i in range(1, self.n + 1):
            for j in range(1, self.n + 1):
                d[i, j] = self.pairwise_distance[i][j]
    
    def clone_shortest_graph(self, double[:, :] d, int[:, :] depth, int[:, :] fa):
        cdef int i, j
        for i in range(1, self.n + 1):
            for j in range(1, self.n + 1):
                d[i, j] = self.pairwise_distance[i][j]
                depth[i, j] = self.shortest_graph_depth[i][j]
                fa[i, j] = self.shortest_graph_fa[i][j]
