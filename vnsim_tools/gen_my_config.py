# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import os
import json
import xml.dom.minidom
import numpy as np

if __name__ == "__main__":
    from utils import RawMyDict
else:
    from .utils import RawMyDict


def process(e_dict, map_data, data_dir, save_dir=None, DEBUG_LEVEL=0):
    data = {}
    n, m = 0, len(map_data["edge_length"])
    e_x, e_y = {}, {}
    for i in range(1, m + 1):
        x, y, l = map_data["edge_length"][str(i)]
        e_x[i] = x
        e_y[i] = y
        n = max(n, max(x, y))
    if DEBUG_LEVEL >= 1:
        print("n=", n, "m=", m)
    
    # Load vertex position
    with open(os.path.join(data_dir, "path.config"), "r") as f:
        data["vertex_position"] = {}
        args = xml.dom.minidom.parse(f)
        paths = args.documentElement.getElementsByTagName("Path")
        def assign_pos(v, x, y):
            if v not in data["vertex_position"]:
                data["vertex_position"][v] = (x, y)
            x0, y0 = data["vertex_position"][v]
            assert np.abs(x - x0) < 1e-6 and np.abs(y - y0) < 1e-6, "vertex location should be unique"
        for path in paths:
            raw_e = int(path.getAttribute("No"))
            my_e = e_dict.raw_to_my(raw_e)
            detail = path.getAttribute("Detail")
            F_or_B, start_pos, end_pos = detail.split(";")
            start_pos, end_pos = start_pos.split(","), end_pos.split(",")
            assign_pos(e_x[my_e], float(start_pos[0]), float(start_pos[1]))
            assign_pos(e_y[my_e], float(end_pos[0]), float(end_pos[1]))
        if DEBUG_LEVEL >= 1:
            for i in range(1, n + 1):
                assert i in data["vertex_position"]
            print("finish loading vertex positions...\n")
    
    # Get park -> edge
    with open(os.path.join(data_dir, "BackParkTable_UTF8.config"), "r") as park_file:
        args = xml.dom.minidom.parse(park_file)
        parks = args.documentElement.getElementsByTagName("Record")
        park_raw_edge = {}
        for park in parks:
            i = int(park.getAttribute("ParkId"))
            p = int(park.getAttribute("PathNo"))
            park_raw_edge[i] = p
        if DEBUG_LEVEL >= 1:
            print("number of parks:", len(park_raw_edge))
            print("finish loading park file...\n")
        
        # Get AGV init vertex
        data["agv_init_vertex"] = {}
        with open(os.path.join(data_dir, "Cars.config"), "r") as agv_init_file:
            args = xml.dom.minidom.parse(agv_init_file)
            vehicles = args.documentElement.getElementsByTagName("AgvConfig")
            for vehicle in vehicles:
                name = vehicle.getAttribute("Id")
                init_park = int(vehicle.getAttribute("InitPark"))
                init_raw_e = park_raw_edge[init_park]
                init_my_e = e_dict.raw_to_my(init_raw_e)
                data["agv_init_vertex"][name] = e_y[init_my_e]
            if DEBUG_LEVEL >= 1:
                print("num_vehicles", len(data["agv_init_vertex"]))
                if DEBUG_LEVEL >= 2:
                    print("agv init vertex")
                    print(data["agv_init_vertex"])
                print("finish loading agv init vertex...\n")
        
        # Get task generate information
        with open(os.path.join(data_dir, "ParksSet.config"), "r") as park_group_set, \
             open(os.path.join(data_dir, "tasks.config"), "r") as task_gen_rules:
            # Get park group
            args = xml.dom.minidom.parse(park_group_set)
            park_sets = args.documentElement.getElementsByTagName("ParkSet")
            park_groups = {}
            for park_set in park_sets:
                k = park_set.getAttribute("Name")
                parks = park_set.getAttribute("Parks").split(";")
                if parks[-1] == "":
                    parks = parks[:-1]
                park_groups[k] = []
                for park in parks:
                    if int(park) in park_groups[k]:
                        continue
                    park_groups[k].append(int(park))
            if DEBUG_LEVEL >= 1:
                print("# park groups: ", len(park_groups))
                if DEBUG_LEVEL >= 2:
                    print("park groups")
                    print(park_groups)
                print("finish loading park groups...\n")
            
            # Get task gen rule
            args = xml.dom.minidom.parse(task_gen_rules)
            agv_tasks = args.getElementsByTagName("Group")
            agv_task_gen_info = {}
            for agv_task in agv_tasks:
                agv_names = agv_task.getAttribute("AgvId").split(",")
                if agv_task.getAttribute("TimeMode") == "Flow":
                    task_start_t = float(agv_task.getAttribute("Time"))
                    task_final_t = 3600 * float(agv_task.getAttribute("FlowDuration"))
                    task_repeat_times = int(agv_task.getAttribute("Times"))
                    task_time_sample = agv_task.getAttribute("TimeSelect")
                    assert task_time_sample in ["Uniform", "Random"]
                else:
                    task_start_t = float(agv_task.getAttribute("Time"))
                    task_final_t = task_start_t
                    task_repeat_times = 1
                    task_time_sample = "Uniform"
                for agv in agv_names:
                    assert agv not in agv_task_gen_info
                    agv_task_gen_info[agv] = {
                        "task_start_t": task_start_t,
                        "task_final_t": task_final_t,
                        "task_repeat_times": task_repeat_times,
                        "task_time_sample": task_time_sample,
                        "task_group": []
                    }
                sub_tasks = agv_task.getElementsByTagName("Task")
                for sub_task in sub_tasks:
                    goal_mode = sub_task.getAttribute("GoalMode")
                    if goal_mode == "PathId":
                        raw_edges = [int(sub_task.getAttribute("Goal"))]
                    elif goal_mode == "ParkId":
                        park = int(sub_task.getAttribute("Goal"))
                        raw_edges = [park_raw_edge[park]]
                    elif goal_mode == "ParkSet":
                        park_group_id = sub_task.getAttribute("Goal")
                        raw_edges = [park_raw_edge[park] for park in park_groups[park_group_id]]
                    goals = [e_y[e_dict.raw_to_my(raw_e)] for raw_e in raw_edges]
                    for agv in agv_names:
                        agv_task_gen_info[agv]["task_group"].append({
                            "priority": int(sub_task.getAttribute("Priority")),
                            "type": sub_task.getAttribute("Type"),
                            "goal_mode": goal_mode,
                            "goal_label": sub_task.getAttribute("Goal"),
                            "goals": goals,
                        })
            data["agv_task_gen_info"] = agv_task_gen_info
            if DEBUG_LEVEL >= 1:
                if DEBUG_LEVEL >= 2:
                    for key in data["agv_task_gen_info"]:
                        print(key)
                        print(data["agv_task_gen_info"][key])
                        break
                print("finish agv_task_gen_info ...\n")
    
        # Read task list
        data["task_list"] = []
        with open(os.path.join(data_dir, "tasks-list.config"), "r") as task_list:
            args = xml.dom.minidom.parse(task_list)
            tasks = args.getElementsByTagName("Group")
            for task in tasks:
                agv_task = {
                    "t": float(task.getAttribute("Time")),
                    "agv": task.getAttribute("AgvId"),
                    "task_group": []
                }
                sub_tasks = task.getElementsByTagName("Task")
                for sub_task in sub_tasks:
                    goal_mode = sub_task.getAttribute("GoalMode")
                    if goal_mode == "ParkId":
                        park = int(sub_task.getAttribute("Goal"))
                        raw_e = park_raw_edge[park]
                    elif goal_mode == "PathId":
                        raw_e = int(sub_task.getAttribute("Goal"))
                    else:
                        raise NotImplementedError
                    my_e = e_dict.raw_to_my(raw_e)

                    agv_task["task_group"].append({
                        "type": sub_task.getAttribute("Type"),
                        "priority": int(sub_task.getAttribute("Priority")),
                        "goal": e_y[my_e],
                    })
                data["task_list"].append(agv_task)
            if DEBUG_LEVEL >= 2:
                print("sample task_list element")
                print(data["task_list"][0])
                print("finish load task-list ...\n")

    # Save files
    fdir = os.path.dirname(__file__) if save_dir is None else save_dir
    with open(os.path.join(fdir, "config_data.json"), "w") as f:
        json.dump(data, f, indent=2)

def validate_task_list_in_task_gen(task_list, task_gen_info):
    from collections import defaultdict
    dd = defaultdict(int)
    agv_list = []
    for task in task_list:
        agv = task["agv"]
        if agv not in agv_list:
            agv_list.append(agv)
        task_repeat_times = task_gen_info[agv]["task_repeat_times"]
        task_start_t = task_gen_info[agv]["task_start_t"]
        task_final_t = task_gen_info[agv]["task_final_t"]
        k = (task["t"] - task_start_t) / (task_final_t / task_repeat_times)
        assert k - round(k) <= 0.01
        k = int(round(k))
        dd[(agv_list.index(agv), k)] += 1
        for subtask, subtask_pool in zip(task["task_group"], task_gen_info[agv]["task_group"]):
            assert subtask["goal"] in subtask_pool["goals"]

    count = 0
    for agv in agv_list:
        for i in range(task_gen_info[agv]["task_repeat_times"]):
            count += 1
            assert dd[(agv_list.index(agv), i)] == 1, f"{agv}, {i}"
    assert count == len(dd), f"count={count}, len(dd)={len(dd)}"

if __name__ == "__main__":
    from load_data import read_map
    import os
    data_dir = "D:/Tsinghua_23_2/AI_AGV/0617workdir/VnSimulator_v2.5/tjsh200/config/newtjsh2"
    e_dict = RawMyDict()
    e_dict.load("./e_dict.json")
    with open("./map_data.json", "r") as f:
        map_data = json.load(f)
    process(e_dict, map_data, data_dir, DEBUG_LEVEL=2)
    with open("./config_data.json", "r") as f:
        config_data = json.load(f)
    data = {}
    data.update(map_data)
    data.update(config_data)
    
    from copy import deepcopy
    with open("construct_graph.json", "r") as f:
        gt_data = json.load(f)
    print("gt keys", gt_data.keys())
    print("gt n=", len(gt_data["vertex_position"]))
    print("my keys", data.keys())
    print()
    for k in data:
        if k not in gt_data:
            continue
        print("diff key=", k, "...")
        d1, d2 = gt_data[k], data[k]
        def check(d1, d2):
            for kk in d1:
                if isinstance(d1[kk], list) and len(d1[kk]) == 0:
                    continue
                if kk not in d2:
                    print(f"gt key [ {kk} ] not in my data")
                    return 0
                v1, v2 = d1[kk], d2[kk]
                if isinstance(v1, list) or isinstance(v1, int) or isinstance(v1, float):
                    if v1 != v2:
                        print(f"key {kk}, gt={v1}, my={v2}")
                        return 0
                elif isinstance(v1, list):
                    if len(v1) != len(v2):
                        print(f"key {kk}, gt={v1}, my={v2}")
                        return 0
                    if len(v1) == 0:
                        continue
                    if not (isinstance(v1[0], list) or isinstance(v1[0], int) or isinstance(v1[0], float)):
                        print(v1[0], "type=", type(v1[0]))
                        raise NotImplementedError
                    if isinstance(v1[0], list):
                        vv1, vv2 = []
                        for oo in v1:
                            vv1.append(tuple(oo))
                        for oo in v2:
                            vv2.append(tuple(oo))
                    else:
                        vv1, vv2 = deepcopy(v1), deepcopy(v2)
                    vv1.sort()
                    vv2.sort()
                    for vv1x, vv2x in zip(vv1, vv2):
                        if vv1x != vv2x:
                            print(f"key {kk}, sorted gt={vv1}, my={vv2}")
                            return 0
            for kk in d2:
                if isinstance(d2[kk], list) and len(d2[kk]) == 0:
                    continue
                if kk not in d1:
                    print("kk=", kk)
                    print(d2[kk])
                    print(f"gt key does not have [ {kk} ] of my data")
                    return 0
            return 1
        if isinstance(d1, dict):
            res = check(d1, d2)
            print("success\n" if res else "fail!\n")
        elif k == "task_list":
            if len(d1) == len(d2):
                res = True
                for xx, yy in zip(d1, d2):
                    if xx != yy:
                        res = False
                        break
            else:
                res = False
            print("success\n" if res else "fail!\n")
        else:
            print("unknown type=", type(d1))
            raise NotImplementedError

    validate_task_list_in_task_gen(gt_data["task_list"], data["agv_task_gen_info"])
