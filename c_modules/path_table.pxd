# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.pair cimport pair
from libcpp.set cimport set
from libcpp.unordered_map cimport unordered_map
from libcpp.vector cimport vector

from utils cimport ivec, StepNode
from graph cimport Graph


cdef class PathTable:
    cdef int n, m
    cdef vector[ivec] clash_relation
    cdef vector[set[StepNode]] occupy_table, clash_table
    cdef long long global_clash_table_stamp
    cdef vector[long long] clash_node_version
    cdef int path_global_counter
    cdef set[int] existed_path
    cdef unordered_map[int, vector[StepNode]] path_buffer
    
    cdef inline int v_to_xid(self, int x):
        return x
    
    cdef inline int e_to_xid(self, int x):
        return self.n + x
    
    cdef inline int xid_to_v(self, int x):
        return x
    
    cdef inline int xid_to_e(self, int x):
        return x - self.n
    
    cdef inline bint xid_is_v(self, int x):
        return x <= self.n

    cdef void clear_clash_and_path(self)
    cdef void add_clash_interval(self, int xid, const StepNode& step_node, int val)
    cdef void add_step_node(self, const StepNode& step_node, int val)
    cdef int store_path(self, const vector[StepNode]&)
    cdef vector[StepNode] access_path(self, int path_id)
    cpdef double query_path_length(self, int path_id)
    cpdef double query_path_reach_time(self, int path_id)
    cdef pair[vector[StepNode], vector[StepNode]] count_collision_on_path(self, int my_agent_id, int path_id)
