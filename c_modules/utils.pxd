# distutils: language=c++
# cython: language_level=3

# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
from libcpp.pair cimport pair
from libcpp.vector cimport vector
from libcpp.memory cimport shared_ptr

ctypedef pair[int, int] ipair
ctypedef pair[double, double] dpair
ctypedef vector[int] ivec
ctypedef vector[double] dvec
ctypedef vector[ipair] iivec
ctypedef vector[dpair] ddvec


cdef extern from "utils.h":
    cdef cppclass EdgeNode:
        int e_id, v_in, v_out
        double cost
        EdgeNode() except +
        EdgeNode(int, int, int, double) except +
    
    cdef cppclass StepNode:
        int loc, agent_id
        double t_l, t_r
        StepNode() except +
        StepNode(int, int, double, double) except +
        bint operator<(const StepNode&) const

cdef extern from "<random>" namespace "std" nogil:
    cdef cppclass mt19937:
        mt19937()
        mt19937(unsigned int seed)
        void seed(unsigned int seed)
    
    cdef cppclass uniform_real_distribution[T]:
        uniform_real_distribution()
        uniform_real_distribution(T a, T b)
        T operator()(mt19937 gen)
    
    cdef cppclass uniform_int_distribution[T]:
        uniform_int_distribution()
        uniform_int_distribution(T a, T b)
        T operator()(mt19937 gen)

cdef extern from "utils.h":
    cdef cppclass RandGen:
        mt19937 gen
        RandGen() except +
        RandGen(unsigned int) except +
        void seed(unsigned int)
        double uniform(double, double)
        int randint(int, int)
    
    cdef cppclass CBSConstraint:
        int loc, agent_id
        double t_l, t_r
        int constraint_type
        CBSConstraint() except +
        CBSConstraint(int, int, double, double, int) except +
        StepNode occupy()
        StepNode forbid()
    
    cdef cppclass CBSConflict:
        int agent_id_0, agent_id_1
        vector[CBSConstraint] constraints_0, constraints_1
        int conflict_type
        CBSConflict() except +
        CBSConflict(int, int, double, double, int) except +
        vector[CBSConstraint] get_constraints(int)
        void edge_to_edge_conflict(int, int, int, int, double, double, double)
        bint operator<(const CBSConflict&) const

    cdef cppclass CBSNode:
        vector[ipair] agent_paths
        vector[shared_ptr[CBSConflict]] conflicts # TODO: copy conflicts
        vector[CBSConstraint] last_constraints
        double path_length_sum
        int conflicted_pairs
        CBSNode* fa
        CBSNode() except +
        void clear()
