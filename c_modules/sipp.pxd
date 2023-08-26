# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.pair cimport pair
from libcpp.set cimport set
from libcpp.vector cimport vector

from graph cimport Graph
from path_table cimport PathTable

from utils cimport ipair, dpair, ivec, dvec, ddvec, EdgeNode, StepNode


ctypedef vector[dvec] sipp_vec
ctypedef pair[bint, ipair] ParentNode
ctypedef vector[vector[ParentNode]] parent_vec


cdef class Planner:
    pass

cdef class SIPP(Planner):
    cdef int n, m
    cdef vector[vector[EdgeNode]] inner_edge_out
    cdef ivec v_xid, e_xid
    cdef double MAX_TIME, SAFE_EPS
    cdef int max_num_safe_intervals
    cdef ivec num_intervals
    cdef vector[long long] interval_version, edge_safe_interval_version
    cdef vector[vector[bint]] interval_is_safe
    cdef vector[ddvec] intervals, edge_safe_intervals
    cdef vector[sipp_vec] arrival_time
    cdef vector[parent_vec] fa
    cdef void refresh_nearby_safe_intreval(self, int x, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version)
    cdef void sipp_vertex_reset(self, int x)
    cdef int kth_interval(self, int x, double t, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version)
    cdef sipp_process(self, int target, double target_stay_time, sipp_vec& f, sipp_vec& g, parent_vec& fa_f, parent_vec& fa_g, dvec& heuristic, const vector[set[StepNode]]& clash_table, const vector[long long]& clash_node_version)
    cdef void update_interval(self, int x, const set[StepNode]& clash_intervals, long long clash_info_version)
    cdef void update_edge_safe_interval(self, int e, double e_len, const set[StepNode]& clash_intervals, long long clash_info_version)
    