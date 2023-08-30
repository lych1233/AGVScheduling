// Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <random>
#include <utility>
#include <vector>

class EdgeNode {
    public:
        int e_id, v_in, v_out;
        double cost;
        EdgeNode() {}
        EdgeNode(int id, int x, int y, double z): e_id(id), v_in(x), v_out(y), cost(z) {}
        ~EdgeNode() {}
};

class StepNode {
    public:
        int loc, agent_id;
        double t_l, t_r;
        StepNode() {}
        StepNode(int loc, int agent_id, double t_l, double t_r):
            loc(loc), agent_id(agent_id), t_l(t_l), t_r(t_r) {}
        ~StepNode() {}
        bool operator<(const StepNode& a) const {
            if (t_l != a.t_l) return t_l < a.t_l;
            if (t_r != a.t_r) return t_r < a.t_r;
            if (agent_id != a.agent_id) return agent_id < a.agent_id;
            return loc < a.loc;
        }
};

class RandGen {
    public:
        std::mt19937 gen;
        RandGen() {}
        RandGen(unsigned int seed):
            gen(std::mt19937(seed)) {}
        ~RandGen() {}
        void seed(unsigned int seed) {
            gen.seed(seed);
        }
        double uniform(double l, double r) {
            return std::uniform_real_distribution<double>(l, r)(gen);
        }
        int randint(int l, int r) {
            return std::uniform_int_distribution<int>(l, r)(gen);
        }
};

class CBSConstraint {
    public:
        int loc, agent_id;
        double t_l, t_r;
        int constraint_type; 
        CBSConstraint() {}
        CBSConstraint(int loc, int agent_id, double t_l, double t_r, int constraint_type):
            loc(loc), agent_id(agent_id), t_l(t_l), t_r(t_r), constraint_type(constraint_type) {}
        ~CBSConstraint() {}
        StepNode occupy() {
            return StepNode(loc, agent_id, t_l, t_r);
        }
        StepNode forbid() {
            return StepNode(loc, -1, t_l, t_r);
        }
};

class CBSConflict {
    public:
        int agent_id_0, agent_id_1;
        std::vector<CBSConstraint> constraints_0, constraints_1;
        int conflict_type; 
        CBSConflict() {}
        ~CBSConflict() {}
        std::vector<CBSConstraint> get_constraints(int i) {
            return i == 0 ? constraints_0 : constraints_1;
        }
        void edge_to_edge_conflict(int agent_0, int agent_1, int xid_0, int xid_1, double t, double e_cost_0, double e_cost_1) { // TODO: add conflict type and constraint_type
            agent_id_0 = agent_0;
            agent_id_1 = agent_1;
            constraints_0.push_back(CBSConstraint(xid_0, agent_0, t, t, -1));
            constraints_1.push_back(CBSConstraint(xid_1, agent_1, t, t, -1));
            conflict_type = -1; // TODO: add conflict type
        }
        bool operator<(const CBSConflict& a) const {
            return conflict_type < a.conflict_type; // TODO: better
        }
};

class CBSNode {
    public:
        std::vector< std::pair<int, int> > agent_paths; // TODO: store all paths
		std::vector< std::shared_ptr<CBSConflict> > conflicts;
        std::vector<CBSConstraint> last_constraints;
        double path_length_sum;
        int conflicted_pairs;
		CBSNode* fa;
        CBSNode(): path_length_sum(0), conflicted_pairs(0), fa(NULL) {}
        ~CBSNode() {}
        void clear() {
            conflicts.clear();
        } // TODO: clear useless information; also clear unknown
};

#endif
