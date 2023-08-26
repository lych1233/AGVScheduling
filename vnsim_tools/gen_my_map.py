# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import os
import json
import numpy as np

if __name__ == "__main__":
    from utils import RawMyDict
else:
    from .utils import RawMyDict


def get_vetices(m, inE):
    """Return a tuple (cnt: number of vertice, dict: End_of_Edge -> Vertex)
    """
    fa = np.zeros(1 + m, dtype=int)
    for i in range(1, 1 + m):
        fa[i] = i

    def getfa(x):
        if x == fa[x]:
            return x
        else:
            t = getfa(fa[x])
            fa[x] = t
            return t

    for e_y in inE:
        if len(inE[e_y]) < 2:
            continue
        e_x0 = inE[e_y][0]
        for e_x in inE[e_y][1:]:
            fa[getfa(e_x)] = getfa(e_x0)
    
    cnt = 0
    e2v = {}
    for i in range(1, 1 + m):
        if getfa(i) == i:
            cnt += 1
            e2v[i] = cnt
    for i in range(1, 1 + m):
        e2v[i] = e2v[getfa(i)]
    return cnt, e2v

def get_my_map(raw_map):
    e_dict = RawMyDict()
    my_map = {}
    for k in raw_map.keys():
        my_map[k] = {}
        for raw_e in raw_map[k]:
            my_e = e_dict.raw_to_my(raw_e)
            if isinstance(raw_map[k][raw_e], list):
                my_map[k][my_e] = []
                for raw_data in raw_map[k][raw_e]:
                    my_map[k][my_e].append(e_dict.raw_to_my(raw_data))
            elif isinstance(raw_map[k][raw_e], float):
                my_map[k][my_e] = raw_map[k][raw_e]
            else:
                raise NotImplementedError("type undefined in transferring the map")
    e_dict.freeze()
    return e_dict, my_map

def validate_graph(e_dict, E_start, E_end, my_map):
    G = {}
    for e in my_map["E_in"]:
        x, y = E_start[e], E_end[e]
        if (x, y) in G:
            print(f"Multiple Edges {x} -> {y}, corresponding to raw edges {e_dict.my_to_raw(G[(x, y)])} and {e_dict.my_to_raw(e)}")
        else:
            G[(x, y)] = e
    for e in my_map["E_cost"]:
        x, y = E_start[e], E_end[e]
        assert np.abs(my_map["E_cost"][G[(x, y)]] - my_map["E_cost"][e]) < 1e-6, "the length should be aligned"

    for e in my_map["E_out"]:
        for e_out in my_map["E_out"][e]:
            assert E_end[e] == E_start[e_out], "end[e] should be equal to start[e_out] for e -> e_out"
    
    print("\nvalidation success!\n")

def graph_connect_block_info(n, E_start, E_end, my_map):
    G = {}
    for e in my_map["E_in"]:
        x, y = E_start[e], E_end[e]
        G[(x, y)] = e
    fa = np.zeros(n + 1, dtype=int)
    for i in range(1, 1 + n):
        fa[i] = i
    def getfa(x):
        if x == fa[x]:
            return x
        else:
            t = getfa(fa[x])
            fa[x] = t
            return t
    for (x, y) in G:
        fa[getfa(x)] = getfa(y)
    block_cnt = 0
    for i in range(1, 1 + n):
        if getfa(i) == i:
            block_cnt += 1
            block_sz = 0
            for j in range(1, 1 + n):
                if getfa(j) == i:
                    block_sz += 1
            print(f"block #{block_cnt}, size = {block_sz}")
    print()

def check_contain_info(n, m, Estart, Eend, VV, VE, EV, EE):
    def fail(v, e):
        for v_clash in VV[v]:
            if v_clash not in EV[e]:
                print(f"fail v={v}, e={e}, v_clash={v_clash}")
                for e in range(1, m + 1):
                    if Eend[e] == v_clash:
                        print("v_clash is end of my e", e)
                        break
                return True
        for e_clash in VE[v]:
            if e_clash not in EE[e]:
                print(f"fail v={v}, e={e}, e_clash={e_clash}")
                return True
        return False
    for e in range(1, m + 1):
        x, y = Estart[e], Eend[e]
        if fail(y, e):
            return False
    return True

def check_VV_in_EV_and_VE_in_EE(my_map):
    VV = my_map["V2V_E_clash"]
    VE = my_map["V_E_clash"]
    EV = {}
    check_counts = 0
    for v in VE:
        for e in VE[v]:
            if e not in EV:
                EV[e] = []
            EV[e].append(v)
    EE = my_map["E_clash"]
    for end_edge in VV:
        for v_clash in VV[end_edge]:
            check_counts += 1
            if (end_edge not in EV) or (v_clash not in EV[end_edge]):
                print(f"fail in end_edge=[ {end_edge} ], v_clash=edge_of_edge[ {v_clash} ]")
                return False
    for end_edge in VE:
        for e_clash in VE[end_edge]:
            check_counts += 1
            if (end_edge not in EE) or (e_clash not in EE[end_edge]):
                print(f"fail in end_edge=[ {end_edge} ], e_clash=edge[ {e_clash} ]")
                return False
    print("check #", check_counts, "clashes")
    return True

def process(raw_map, save_dir=None, DEBUG_LEVEL=0):
    # Note in default, v, e are my_v, my_e; raw_v, raw_e should be specified explicitly
    data = {}

    # Get edges
    e_dict, my_map = get_my_map(raw_map)

    m = e_dict.total_entries()
    if DEBUG_LEVEL >= 1:
        print("number of edges", m)
    
    # Get Vertices
    n, Eend_toV = get_vetices(m, my_map["E_in"])
    if DEBUG_LEVEL >= 1:
        print("number of vertices", n)
        print()
    
    # Label endpoints of edges
    Estart_toV = {}
    for e in my_map["E_in"]:
        if len(my_map["E_in"][e]) == 0:
            n += 1
            Estart_toV[e] = n
            if DEBUG_LEVEL >= 2:
                print(f"edge {e_dict.my_to_raw(e)} does not have a income degree")
        else:
            Estart_toV[e] = Eend_toV[my_map["E_in"][e][0]]
            if DEBUG_LEVEL >= 1:
                for e_in in my_map["E_in"][e]:
                    assert Eend_toV[e_in] == Estart_toV[e], "every inedge of the same edge should have the same edgepoint vertex"
    data["edge_length"] = {}
    for e in my_map["E_cost"]:
        data["edge_length"][e] = (Estart_toV[e], Eend_toV[e], my_map["E_cost"][e])
    if DEBUG_LEVEL >= 1:
        print()
        validate_graph(e_dict, Estart_toV, Eend_toV, my_map)
        print()
    if DEBUG_LEVEL >= 2:
        graph_connect_block_info(n, Estart_toV, Eend_toV, my_map)
        print()
    
    # Get clashes
    clash_VV, clash_VE = {v: [] for v in range(1, n + 1)}, {v: [] for v in range(1, n + 1)}
    clash_EV, clash_EE = {e: [] for e in range(1, m + 1)}, {e: [] for e in range(1, m + 1)}
    VV_symmetry = True
    for e in my_map["V2V_E_clash"]:
        v = Eend_toV[e]
        for e_clash in my_map["V2V_E_clash"][e]:
            try:
                assert e in my_map["V2V_E_clash"][e_clash]
            except:
                VV_symmetry = False
            if Eend_toV[e_clash] not in clash_VV[v]:
                clash_VV[v].append(Eend_toV[e_clash])
    
    if not VV_symmetry:
        print("WARNING! vertex to vertex clash is not symmetric")
    
    for e in my_map["V_E_clash"]:
        v = Eend_toV[e]
        for e_clash in my_map["V_E_clash"][e]:
            if e_clash not in clash_VE[v]:
                clash_VE[v].append(e_clash)
    for v in clash_VE:
        for e in clash_VE[v]:
            clash_EV[e].append(v)
    
    for e in my_map["E_clash"]:
        for e_clash in my_map["E_clash"][e]:
            clash_EE[e].append(e_clash)
    
    if DEBUG_LEVEL >= 2:
        print("check_VV_in_EV_and_VE_in_EE? (V are different w.r.t which edge end)")
        if (check_VV_in_EV_and_VE_in_EE(my_map)):
            print("YES!")
        else:
            print("NO!")
        print("finish")

    data["vertex_clash"] = {}
    for v in clash_VV:
        data["vertex_clash"][v] = []
        assert  v <= n
        for v_clash in clash_VV[v]:
            data["vertex_clash"][v].append(("V", v_clash))
        for e_clash in clash_VE[v]:
            data["vertex_clash"][v].append(("E", e_clash))
    data["edge_clash"] = {}
    for e in my_map["E_clash"]:
        data["edge_clash"][e] = []
        if e in clash_EV:
            for v_clash in clash_EV[e]:
                data["edge_clash"][e].append(("V", v_clash))
        for e_clash in clash_EE[e]:
            data["edge_clash"][e].append(("E", e_clash))
    
    my_vv_symmetry = True
    for v in data["vertex_clash"]:
        for t, o in data["vertex_clash"][v]:
            if t == "V":
                try:
                    assert ("V", v) in data["vertex_clash"][o], "VV shoule be symmetric"
                except:
                    my_vv_symmetry = False
            else:
                assert ("V", v) in data["edge_clash"][o], "VE shoule be symmetric"
    if not my_vv_symmetry:
        print("WARNING! in the new graph, vertex-vertex clash is not symmetric")
    my_ee_symmetry = True
    for e in data["edge_clash"]:
        for t, o in data["edge_clash"][e]:
            if t == "V":
                assert ("E", e) in data["vertex_clash"][o], "EV shoule be symmetric"
            else:
                try:
                    assert ("E", e) in data["edge_clash"][o], "EE shoule be symmetric"
                except:
                    my_ee_symmetry = False
    if not my_ee_symmetry:
        print("WARNING! in the new graph, edge-edge clash is not symmetric")
    if DEBUG_LEVEL >= 1:
        print("check clash symmetry, success!")

    # Save files
    fdir = os.path.dirname(__file__) if save_dir is None else save_dir
    e_dict.save(os.path.join(fdir, "e_dict.json"))        
    with open(os.path.join(fdir, "map_data.json"), "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    from load_data import read_map
    import os
    parent_dir = "D:/Tsinghua_23_2/AI_AGV/0617workdir"
    raw_map = read_map(os.path.join(parent_dir, "VnSimulator_v2.5", "map_tjsh"))
    process(raw_map, DEBUG_LEVEL=2)
    
    from copy import deepcopy
    with open("construct_graph.json", "r") as f:
        gt_data = json.load(f)
    with open("map_data.json", "r") as f:
        map_data = json.load(f)
    print("gt keys", gt_data.keys())
    print("gt n=", len(gt_data["vertex_position"]))
    print("map keys", map_data.keys())
    print()
    for k in map_data:
        if k not in gt_data:
            continue
        print("diff key=", k, "...")
        d1, d2 = gt_data[k], map_data[k]
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
        else:
            print("unknown type=", type(d1))
            raise NotImplementedError
