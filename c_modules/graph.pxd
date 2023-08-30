# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.vector cimport vector

from utils cimport ivec, dvec, EdgeNode


cdef class Graph:
    cdef int n, m
    cdef vector[dvec] pairwise_distance
    cdef vector[ivec] shortest_graph_depth, shortest_graph_fa
    cdef vector[vector[EdgeNode]] edge_in, edge_out
    cdef vector[EdgeNode] edges
    
    cdef void single_source_dijkstra(self, int s, dvec& d, ivec& depth, ivec &fa)
    cpdef graph_query_nearby_vertices(self, int neighbor_size, int[:, :] query_v, int[:, :] query_depth, int[:, :] query_fa) # TODO :remove
