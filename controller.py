# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
import argparse
import json
import os
import time
import xml.dom.minidom
from collections import deque

import numpy as np

from vnsim_tools.renderer import VnSimulator
from vnsim_tools.load_data import read_map
from vnsim_tools.utils import RawMyDict
from mapf_lns2_solver import MAPF_LNS2_Solver


def process_map(args):
    cache_dir = os.path.join(os.getcwd(), "aux_data")
    os.makedirs(cache_dir, exist_ok=True)
    if args.refresh_data:
        map_path = os.path.join(args.sim_dir, args.map_dir)
        raw_map = read_map(map_path)
        from vnsim_tools.gen_my_map import process
        process(raw_map, save_dir=cache_dir)
    e_dict = RawMyDict()
    e_dict.load(os.path.join(cache_dir, "e_dict.json"))
    with open(os.path.join(cache_dir, "map_data.json"), "r") as f:
        map_data = json.load(f)
    return e_dict, map_data

def process_config(args, e_dict, map_data):
    cache_dir = os.path.join(os.getcwd(), "aux_data")
    if args.refresh_data:
        try:
            with open(os.path.join(args.sim_dir, args.config_dir, args.config_file), "r") as f:
                xml_args = xml.dom.minidom.parse(f)
                fileconfigs = xml_args.getElementsByTagName("FileConfig")
                for fileconfig in fileconfigs:
                    fileconfig_dir = fileconfig.getAttribute("Directory")
                    break
                config_data_dir = os.path.join(args.sim_dir, args.config_dir, fileconfig_dir)
        except:
            print(f'\n========== config file **{args.config_file}** not in UTF8 format, try to load from **{args.config_file.split(".config")[0] + "_UTF8.config"}** ==========\n')
            with open(os.path.join(args.sim_dir, args.config_dir, args.config_file.split(".config")[0] + "_UTF8.config"), "r") as f:
                xml_args = xml.dom.minidom.parse(f)
                fileconfigs = xml_args.getElementsByTagName("FileConfig")
                for fileconfig in fileconfigs:
                    fileconfig_dir = fileconfig.getAttribute("Directory")
                    break
                config_data_dir = os.path.join(args.sim_dir, args.config_dir, fileconfig_dir)
        from vnsim_tools.gen_my_config import process
        process(e_dict, map_data, config_data_dir, save_dir=cache_dir)
    with open(os.path.join(cache_dir, "config_data.json"), "r") as f:
        config_data = json.load(f)
    # path from xml
    return config_data

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim_dir", default="VnSimulator_v2.5", type=str)
    parser.add_argument("--map_dir", default="map_tjsh", type=str)
    parser.add_argument("--config_dir", default="tjsh", type=str)
    parser.add_argument("--config_file", default="CC.tjsh.config", type=str)
    parser.add_argument("--port", default="64801", type=str)
    parser.add_argument("--speedup", default=1, type=int)
    parser.add_argument("--refresh_data", default=False, action="store_true")
    parser.add_argument("--vn_sim_dt", default=0.1, type=float)

    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--target_stay_time", default=35.0, type=float)
    parser.add_argument("--sipp_max_time", default=1000.0, type=float)
    parser.add_argument("--sipp_safe_eps", default=0.1, type=float)
    parser.add_argument("--mapf_lns_max_steps", default=500, type=int)
    parser.add_argument("--mapf_lns_max_time", default=-1, type=float)
    return parser.parse_args()
        

class Controller:
    def __init__(self, args, e_dict, data):
        self.e_dict = e_dict
        self.data = data
        self.mapf_lns2_solver = MAPF_LNS2_Solver(args, data)
        self.target_stay_time = args.target_stay_time
        self.sipp_max_time = args.sipp_max_time
        self.vn_sim_dt = args.vn_sim_dt
        self.map = self.mapf_lns2_solver.map
        self.agv_names = []
        for agv in self.data["agv_init_vertex"]:
            self.agv_names.append(agv)
        self.AGV = len(self.agv_names)
        self.agv_names.sort()
        self.agv_initial_vertices = np.array([
            self.data["agv_init_vertex"][self.agv_names[i]] for i in range(self.AGV)
        ])
        self.vn_sim = VnSimulator(args.sim_dir, config_dir=args.config_dir, config_file=args.config_file, port=args.port, speedup=args.speedup, use_gui=True)
        self.failed_plannings = 0
    
    def get_agv_id(self, agv_name):
        return self.agv_names.index(agv_name)

    def _sep_Position(self, Position):
        e, t = Position.split("@")
        return int(e), float(t)

    def get_mapf_solution(self, mapf_probs):
        start_vertices, start_edges, goal_vertices, start_endpoints, start_time, target_stay_time = [], [], [], [], [], []
        for i in range(self.AGV):
            agv = self.agv_names[i]
            prob = mapf_probs[agv]
            start_vertices.append(prob["start_vertex"])
            start_edges.append(prob["start_edge"])
            goal_vertices.append(prob["goal_vertex"])
            start_endpoints.append(prob["start_endpoint"])
            start_time.append(prob["start_time"])
            target_stay_time.append(prob["target_stay_time"])
        conflict_graph, solutions = self.mapf_lns2_solver.MAPF(start_vertices, start_edges, goal_vertices, start_endpoints, start_time, target_stay_time)
        if conflict_graph.sum() > 0:
            goback_list = np.random.permutation(self.AGV)
            goback_list_1, goback_list_2 = [], []
            for agv_i in goback_list:
                if conflict_graph.sum(-1)[agv_i] > 0:
                    goback_list_1.append(agv_i)
                else:
                    goback_list_2.append(agv_i)
            goback_list = goback_list_1 + goback_list_2
            for agv_i in goback_list:
                goal_vertices[agv_i] = self.agv_initial_vertices[i]
                target_stay_time[agv_i] = self.target_stay_time
                conflict_graph, solutions = self.mapf_lns2_solver.MAPF(start_vertices, start_edges, goal_vertices, start_endpoints, start_time, target_stay_time)
                if conflict_graph.sum() == 0:
                    break
        try:
            assert conflict_graph.sum() == 0
        except:
            self.failed_plannings += 1
        return solutions

    def execute(self, solutions):
        propose_reach = []
        propose_goal_v = []
        propose_final_v = []
        
        states_before = self.vn_sim._get_agv_states()
        tasks_before = self.vn_sim._get_agv_tasks()
        for i in range(self.AGV):
            agv = self.agv_names[i]
            single_task_id = int(states_before[agv]["SingleTaskId"])
            task_group_id = int(states_before[agv]["TaskGroupId"])
            if single_task_id > 0:
                assert task_group_id == tasks_before[agv]["TaskGroupId"], f"task_group_id={task_group_id}, tasks_before[agv]={tasks_before[agv]['TaskGroupId']}"
                raw_goal_e = tasks_before[agv]["Tasks"][single_task_id]["Goal"]
                my_goal_e = self.e_dict.raw_to_my(raw_goal_e)
                my_goal_v = self.map.e_y[my_goal_e]
            else:
                my_goal_v = -1
            propose_goal_v.append(my_goal_v)
        
        final_t = self.sipp_max_time
        for i in range(self.AGV):
            final_t = min(final_t, solutions[i][-1][1])
        propose_max_exec_t = self.sipp_max_time
        for i in range(self.AGV):
            propose_max_exec_t = min(propose_max_exec_t, solutions[i][-1][2])
        
        execute_list = []
        control_path_table = {}
        for i in range(self.AGV):
            prev_t = -1e100
            control_paths = []
            final_e = -1
            for e, t_l, t_r in solutions[i]:
                if e == -1 or t_l > final_t:
                    break
                if e > 0:
                    if prev_t + self.vn_sim_dt < t_l:
                        execute_list.append((t_l, self.agv_names[i]))
                        control_path_table[(t_l, self.agv_names[i])] = [e]
                    else:
                        control_path_table[execute_list[-1]].append(e)
                prev_t = t_r
                final_e = e
            if final_e == -1:
                raw_cur_e, e_t = self._sep_Position(states_before[agv]["Position"])
                my_cur_e = self.e_dict.raw_to_my(raw_cur_e)
                assert e_t == 0 or e_t == 1
                final_v = self.map.e_x[my_cur_e] if e_t == 0 else self.map.e_y[my_cur_e]
            else:
                final_e = self.e_dict.raw_to_my(final_e)
                final_v = self.map.e_y[final_e]
            propose_final_v.append(final_v)
            propose_reach.append(propose_final_v[i] == propose_goal_v[i])
        execute_list.sort()
        execute_cur = 0       

        agv_current_edge = []
        init_states = self.vn_sim._get_agv_states()
        for agv in self.agv_names:
            e, e_t = self._sep_Position(init_states[agv]["Position"])
            if e_t > 0:
                agv_current_edge.append(e)
            else:
                agv_current_edge.append(-1)
        agv_edges_to_exec = [deque() for i in range(self.AGV)]

        steps = 0
        max_step_limit = int(10 * propose_max_exec_t / self.vn_sim_dt)
        max_time_limit = 2 * propose_max_exec_t
        offset_t = float(self.vn_sim._get_agv_states()[self.agv_names[0]]["TimeStamp"]) + self.vn_sim_dt
        cur_runtime_t = float(self.vn_sim._get_agv_states()[self.agv_names[0]]["TimeStamp"]) - offset_t
        print("max_step_limit", max_step_limit, "max_time_limit", max_time_limit, "final_t", final_t)
        while steps <= max_step_limit and cur_runtime_t <= max_time_limit:
            agv_update_list = []
            while execute_cur < len(execute_list):
                exec_t, agv = execute_list[execute_cur]
                states = self.vn_sim._get_agv_states()
                sim_t = float(states[agv]["TimeStamp"])
                if sim_t >= offset_t + exec_t:
                    control_paths = control_path_table[(exec_t, agv)]
                    print("\nadd", agv, states[agv], control_paths)
                    print("exec_t=", exec_t, "offset_t=", offset_t, "sim_t=", sim_t)
                    i = self.get_agv_id(agv)
                    if i not in agv_update_list:
                        agv_update_list.append(i)
                    for e in control_paths:
                        agv_edges_to_exec[i].append(e)
                    print("agv:", self.agv_names[i], "agv_edges_to_exec[i]", agv_edges_to_exec[i])
                    print("\n\n")
                else:
                    break
                execute_cur += 1
            states = self.vn_sim._get_agv_states()
            sim_control_paths = {}
            for i in range(self.AGV):
                if len(agv_edges_to_exec[i]) == 0:
                    continue
                agv = self.agv_names[i]
                e, e_t = self._sep_Position(states[agv]["Position"])
                if e_t > 0:
                    if agv_current_edge[i] != e:
                        agv_current_edge[i] = e
                        if e not in list(agv_edges_to_exec[i]):
                            print("agv", agv)
                            print(self.vn_sim._get_agv_states()[agv])
                            print(e)
                            print(agv_edges_to_exec[i])
                            raise RuntimeError
                        while agv_edges_to_exec[i][0] != e:
                            agv_edges_to_exec[i].popleft()
                        agv_edges_to_exec[i].popleft()
                if i in agv_update_list:
                    print("\ncontrol agv", agv, "\n", states[agv], agv_edges_to_exec[i])
                    sim_control_paths[agv] = agv_edges_to_exec[i]
            if len(sim_control_paths):
                self.vn_sim._send_path(states, sim_control_paths)
                time.sleep(self.vn_sim_dt)

            try:
                assert execute_cur == len(execute_list)
                for i in range(self.AGV):
                    assert len(agv_edges_to_exec[i]) == 0
                for agv in states:
                    e, e_t = self._sep_Position(states[agv]["Position"])
                    assert e_t == 0 or e_t == 1
                break
            except:
                steps += 1
                cur_runtime_t = float(self.vn_sim._get_agv_states()[self.agv_names[0]]["TimeStamp"]) - offset_t
                time.sleep(0.1 * self.vn_sim_dt)
            if steps % 30 == 0:
                print("\nstep=", steps, "/", max_step_limit, "\ncur_runtime_t=", cur_runtime_t, "/", max_time_limit, "\n\nstates=\n", self.vn_sim._get_agv_states(), "\ntasks=\n", self.vn_sim._get_agv_tasks())
                print("execute_cur, len(execute_list)", execute_cur, len(execute_list))
                for i in range(self.AGV):
                    print("agv", self.agv_names[i], "agv_edges_to_exec[i]", agv_edges_to_exec[i])
                print("\n\n")
        
        print("finish")
        print("task_before", tasks_before)
        print("states", self.vn_sim._get_agv_states())
        
        states_after = self.vn_sim._get_agv_states()

        sim_final_v = []
        sim_reach = []
        for i in range(self.AGV):
            agv = self.agv_names[i]
            final_e, e_t = self._sep_Position(states_after[agv]["Position"])
            assert e_t == 0 or e_t == 1
            my_e = self.e_dict.raw_to_my(final_e)
            sim_final_v.append(self.map.e_x[my_e] if e_t == 0.0 else self.map.e_y[my_e])
            sim_reach.append(
                sim_final_v[i] == propose_goal_v[i]
            )
        
        wait_load_start_time = time.time()
        while time.time() - wait_load_start_time < self.target_stay_time:
            sim_finish = []
            states_after = self.vn_sim._get_agv_states()
            for i in range(self.AGV):
                agv = self.agv_names[i]
                sim_finish.append(
                    states_before[agv]["SingleTaskId"] != states_after[agv]["SingleTaskId"] or states_before[agv]["TaskGroupId"] != states_after[agv]["TaskGroupId"]
                )
            if np.all(np.array(sim_finish) == np.array(propose_reach)):
                break
        
        if not np.all(np.array(sim_finish) == np.array(propose_reach)):
            print(states)
            print("sim_final_v", sim_final_v)
            print("propose_final_v", propose_final_v)
            print("sim_reach", sim_reach)
            print("sim_finish", sim_finish)
            print("propose_reach", propose_reach)
    
    def run(self):
        while True:
            states = self.vn_sim._get_agv_states()
            try:
                for agv in self.agv_names:
                    assert states[agv]["SingleTaskId"] == "1"
                break
            except:
                pass
            time.sleep(0.1)
        
        solution_history_info = []
        while True:
            states = self.vn_sim._get_agv_states()
            tasks = self.vn_sim._get_agv_tasks()
            try:
                for agv in states:
                    assert states[agv]["SingleTaskId"] == "0"
                break
            except:
                pass
            print("\nstates = ", states)
            print("\ntasks = ", tasks)
            info = {
                "states": states,
                "tasks": tasks,
            }
            assert len(states) == self.AGV, f"states={states} \n tasks={tasks}"
            mapf_probs = {}
            for agv in states:
                state = states[agv]
                prob = {}
                prob["raw_e"], prob["e_t"] = self._sep_Position(state["Position"])
                if int(state["SingleTaskId"]) > 0:
                    task = tasks[agv]
                    assert int(state["TaskGroupId"]) == task["TaskGroupId"], f"unknown task group id: {state['TaskGroupId']}"
                    prob["raw_goal_e"] = int(task["Tasks"][int(state["SingleTaskId"])]["Goal"])
                else:
                    prob["raw_goal_e"] = -1
                
                prob["my_e"] = self.e_dict.raw_to_my(prob["raw_e"])
                if prob["raw_goal_e"] == -1:
                    prob["goal_vertex"] = -1
                    prob["target_stay_time"] = 0.5 * self.sipp_max_time
                else:
                    prob["goal_vertex"] = self.map.e_y[self.e_dict.raw_to_my(prob["raw_goal_e"])]
                    prob["target_stay_time"] = self.target_stay_time
                assert prob["e_t"] >= 0 and prob["e_t"] <= 1
                if prob["e_t"] == 1:
                    prob["start_vertex"] = self.map.e_y[prob["my_e"]]
                    prob["start_edge"] = -1
                    prob["start_endpoint"] = prob["start_vertex"]
                    prob["start_time"] = 0.0
                else:
                    prob["start_vertex"] = -1
                    my_e = prob["my_e"]
                    prob["start_edge"] = my_e
                    prob["start_endpoint"] = self.map.e_y[my_e]
                    prob["start_time"] = self.map.e_z[my_e] * (1 - prob["e_t"])
                mapf_probs[agv] = prob
            
            assert len(mapf_probs) == self.AGV
            solutions = self.get_mapf_solution(mapf_probs)
            for solution in solutions:
                assert solution[-1][0] == -1
                for i in range(len(solution) - 1):
                    if solution[i][0] == -1:
                        continue
                    solution[i] = (self.e_dict.my_to_raw(solution[i][0]), solution[i][1], solution[i][2])
            info["solutions"] = solutions
            solution_history_info.append(info)
            self.execute(solutions)
        
        with open("solution_history_info.json", "w") as f:
            json.dump(solution_history_info, f, indent=2)
    
    def close(self):
        self.vn_sim.close()
    

if __name__ == "__main__":
    args = get_args()
    with open("config.json", "r") as f:
        config = json.load(f)
        for k in config:
            setattr(args, k, config[k])
    args.sim_dir = str(os.path.join(os.getcwd(), args.sim_dir))
    print("config")
    for k, v in vars(args).items():
        print(k, " " * (30 - len(str(k))) + "||" + " " * 10, v)
    e_dict, map_data = process_map(args)
    config_data = process_config(args, e_dict, map_data)
    data = {}
    data.update(map_data)
    data.update(config_data)
    with open(f"{args.config_dir}.json", "w") as f:
        json.dump(data, f, indent=2)
    
    controller = Controller(args, e_dict, data)
    controller.run()
    controller.close()
    print("failed_plannings", controller.failed_plannings)
