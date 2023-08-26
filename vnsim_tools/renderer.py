# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the GPL-style license found in the
# LICENSE file in the root directory of this source tree. 
import json
import os
import requests
import subprocess
import time
import xml.etree.ElementTree as ET

from requests.exceptions import ConnectionError, ReadTimeout

from .load_data import read_map


class VnSimulator(object):
    def __init__(self, sim_dir, config_dir="tjsh", config_file="CC.tjsh.config", port="64801", speedup=1, use_gui=True):
        self.url = f"http://127.0.0.1:{port}/jsonrpc"

        config_name = f"{config_dir}/port-{port}_speed-{speedup}x_gui-{use_gui}.config"
        config_path = os.path.join(sim_dir, config_name)

        if not os.path.exists(config_path):
            tree = ET.parse(os.path.join(sim_dir, f"{config_dir}/{config_file}"))
            tree.getroot()[0][3].set("DiagnosisPort", port)
            tree.getroot()[0][3].set("GuiHide", "false" if use_gui else "true")
            tree.getroot()[0][3].set("SpeedUp", str(speedup))
            tree.write(config_path, encoding="utf-8", xml_declaration=True)     
        
        self.sim = subprocess.Popen(f"cd {sim_dir} && VnSimulator.exe {config_name}", stdout=subprocess.DEVNULL , stderr=subprocess.DEVNULL, shell=True)

        self._wait_start()
    
    def _wait_start(self):
        # try to connect
        counter = 0
        while True:
            try:
                self._reset_sim()
                return
            except ConnectionError:
                counter += 1
                if counter == 5:
                    raise RuntimeError("tried 5 times, please try later...")
                print(f"Connect Timeout, Remote is not ready, try in 10 secondes... Tried times: {counter}")
                time.sleep(10)
                continue
            except ReadTimeout:
                counter += 1
                if counter == 5:
                    raise RuntimeError("tried 5 times, please try later...")
                print(f"Read Timeout, Remote is not ready, try in 10 secondes... Tried times: {counter}")
                time.sleep(10)
                continue

    def _reset_sim(self):
        api_data = {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "Reset",
            "params": [],
        }
        info = requests.post(url=self.url, data=json.dumps(api_data), timeout=10)
        return info
    
    def _get_agv_states(self):
        """Return:
        {
            "01" : {
                "Position" : "234@0.3",
                "TaskGroupId" : "345678",
                "SingleTaskId" : "1",
                "TimeStamp" : 10.5
            },
            "02" : {
                "Position" : "234@0.1",
                "TaskGroupId" : "345679",
                "SingleTaskId" : "1",
                "TimeStamp" : 10.5          
            }
        }
        """
        api_data = {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "GetAllAgvStatus",
            "params": [],
        }
        res = requests.post(url=self.url, data=json.dumps(api_data), timeout=10)
        json_data = res.json()
        if "result" not in json_data:
            print(json_data.keys())
            exit(0)
        else:
            return json_data["result"]["AgvStatusList"] # TODO: remove
        # return res.json()["result"]["AgvStatusList"]

    def _get_agv_tasks(self):
        """Return tasks:
        {"01":
            "AssignAgv": "01",
            "TaskGroupId": 1,
            "Tasks": {
                1: {
                    "Goal": 7077,
                    "SingleTaskId": 1,
                    "type": "get"
                },
                2: {
                    "Goal": 4349,
                    "SingleTaskId": 2,
                    "type": "put"
                }
                3: {
                    "Goal": 7148,
                    "SingleTaskId": 3,
                    "type": "move"
                }
            },
        "02": ...
        }
        """
        api_data = {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "GetTaskToExecute",
            "params": [],
        }
        res = requests.post(url=self.url, data=json.dumps(api_data), timeout=10)
        task_list = res.json()['result']['TaskGroupList']
        tasks = {}
        if task_list:
            for task in task_list:
                agv = task["AssignAgv"]
                tasks[agv] = {
                    "AssignAgv": agv,
                    "TaskGroupId": task["TaskGroupId"],
                    "Tasks": {},
                }
                for subtask in task["Tasks"]:
                    id = subtask["SingleTaskId"]
                    tasks[agv]["Tasks"][id] = subtask
        return tasks
    
    def _send_path(self, states, paths):
        """Send control path
        Input:
            states: sim_states
            paths: {"01": [1, 2, 3]}
        "params": [
            {
                "01": {
                    "TaskGroupId": 345678,
                    "SingleTaskId": 1,
                    "CtrlPath": "7146,7140"
                },
                "02": {
                    "TaskGroupId": 345679,
                    "SingleTaskId": 1,
                    "CtrlPath": "7146,7140"
                }
            }
        ]
        """
        data = {
            "jsonrpc": "2.0",
            "id": 2,
            "method": "SendControlPath",
            "params": [{}]
        }
        for agv in paths:
            data["params"][0].update({
                agv: {
                    "TaskGroupId": int(states[agv]["TaskGroupId"]),
                    "SingleTaskId": int(states[agv]["SingleTaskId"]),
                    "CtrlPath": ",".join([str(e) for e in paths[agv]])
                }
            })
        data = json.dumps(data)
        info = requests.post(url=self.url, data=data, timeout=10)
        return info

    def reset(self):
        self._reset_sim()
        res = self._get_agv_states()
        return res
    
    def get_timestamp(self):
        agv_states = self._get_agv_states()
        for agv in agv_states:
            return float(agv_states[agv]["TimeStamp"])
        return 0.0

    def close(self):
        time.sleep(5)
        subprocess.call(["taskkill", "/F", "/T", "/PID", str(self.sim.pid)])
        