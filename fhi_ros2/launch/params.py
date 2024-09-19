#
# Copyright 2024 Fraunhofer Italia Research
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import fcntl
import os
import subprocess
import time
from typing import Any, Dict
from ros2param.api import call_list_parameters, call_get_parameters, get_value
from ros2cli.node.direct import DirectNode
from ..utils import remove_none_values


NODE_NOT_FOUND = "Node not found"
PARAMETER_NOT_SET = "Parameter not set"


class ParameterError(Exception):
    def __init__(self, node_name: str, parameter_name: str, message: str):
        super().__init__(
            f"An error has occurred getting parameter '{parameter_name}' for node '{node_name}': '{message}'"
        )


class ParameterNotSetError(ParameterError):
    def __init__(self, node_name: str, parameter_name: str, message: str):
        super().__init__(node_name, parameter_name, message)


class ParameterTimeoutError(ParameterError):
    def __init__(self, node_name: str, parameter_name: str):
        super().__init__(node_name, parameter_name, "timeout")


class ParametersTimeoutError(ParameterError):
    def __init__(self, node_name: str):
        super().__init__(node_name, "", "timeout")


class SystemWideMutex:
    def __init__(self, filename):
        self.filename = filename
        self.file = None
        self.acquired = False

    def _create_lock_file(self):
        if not os.path.exists(self.filename):
            open(self.filename, "a").close()

    def acquire(self, timeout=float("inf")) -> bool:
        self._create_lock_file()
        self.file = open(self.filename, "r+")
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.acquired = True
                return True
            except (IOError, OSError):
                pass

        self.file.close()
        return False

    def release(self):
        if self.acquired:
            fcntl.flock(self.file, fcntl.LOCK_UN)
            self.file.close()
            self.acquired = False

    def __del__(self):
        self.release()


def create_nested_dict(flat_dict):
    nested_dict = {}
    for key, value in flat_dict.items():
        parts = key.split(".")
        d = nested_dict
        for part in parts[:-1]:
            if part not in d:
                d[part] = {}
            d = d[part]
        d[parts[-1]] = value
    return nested_dict


def flatten_nested_dict(nested_dict, parent_key="", sep="."):
    items = []
    for k, v in nested_dict.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_nested_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def dump(node_name: str) -> Dict[str, Any]:
    with DirectNode(None) as node:
        parameter_names = sorted(call_list_parameters(node=node, node_name=node_name))
        response = call_get_parameters(
            node=node, node_name=node_name, parameter_names=parameter_names
        )
        parameter_values = [get_value(parameter_value=i) for i in response.values]
        dict = create_nested_dict(
            {name: value for name, value in zip(parameter_names, parameter_values)}
        )
        return dict


def wait_for_parameter(node_name: str, parameter_name: str, timeout: float = 5.0):
    start_time = time.time()
    while True:
        pipe = subprocess.Popen(
            f"ros2 param get --hide-type {node_name} {parameter_name}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        out, err = (o.decode() for o in pipe.communicate())
        if err.strip() == PARAMETER_NOT_SET:
            raise ParameterNotSetError(node_name, parameter_name, PARAMETER_NOT_SET)
        elif err.strip() == NODE_NOT_FOUND:
            pass
        elif not out == "" and not out.strip() == "None":
            return out

        if time.time() - start_time > timeout:
            raise ParameterTimeoutError(node_name, parameter_name)
        time.sleep(0.05)


def wait_for_parameters(node_name: str, timeout: float = 10.0) -> Dict[str, Any]:
    """Gets the parameters of a node as dict

    Args:
        node_name (str): full name of the node

    Returns:
        Dict[str, Any]: dict containing dump of node parameters
    """
    start_time = time.time()
    while True:
        try:
            return remove_none_values(dump(node_name))
        except RuntimeError as e:
            if time.time() - start_time > timeout:
                raise ParametersTimeoutError(node_name)
