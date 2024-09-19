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
from dataclasses import is_dataclass
import inspect
import os
import shutil
import sys
from typing import Any, List, Dict
from ament_index_python.packages import get_package_share_directory


def merge_dict(d1: dict, d2: dict) -> dict:
    return {**d1, **d2}


def find_files_by_extension(
    directory: str, extension: str, exclude_file_start: str = ""
):
    res = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.startswith(exclude_file_start):
                continue
            if file.endswith(f".{extension}"):
                res.append(os.path.join(root, file))
    return res


class COLORS:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def custom_print(
    string: str, color: COLORS = COLORS.ENDC, indentation=0, print_fcn=print
):
    result_string = color + " " * indentation + string + COLORS.ENDC
    print_fcn(result_string)


class PythonLogger:
    def __init__(self) -> None:
        pass

    def warn(self, string: str):
        return custom_print(string, COLORS.WARNING)

    def error(self, string: str):
        return custom_print(string, COLORS.FAIL)


def get_src_path(package_name: str, print_fcn=custom_print):
    ws_path_found = False
    try:
        ws_path = os.environ["ROS_WS_DIR"]
        ws_path_found = True
    except KeyError:
        print_fcn(
            "[from_share_to_src] Unable to find ROS_WS_DIR environment variable. Using current path.",
            COLORS.WARNING,
        )

    try:
        ws_path = os.environ["ROS2_WS_DIR"]
        ws_path_found = True
    except KeyError:
        print_fcn(
            "[from_share_to_src] Unable to find ROS2_WS_DIR environment variable. Using current path.",
            COLORS.WARNING,
        )

    if not ws_path_found:
        ws_path = os.getcwd()
        sub = [f.path.split("/")[-1] for f in os.scandir(ws_path) if f.is_dir()]
        if not "src" in sub and not "src" == ws_path.split("/")[-1]:
            print_fcn(
                "[from_share_to_src] Unable to find ws_path.",
                COLORS.FAIL,
            )
            sys.exit(0)

    for root, dirs, files in os.walk(ws_path):
        splitted = root.split("/")
        if (
            root.endswith(package_name)
            and not "log" in splitted
            and not "build" in splitted
            and not "install" in splitted
            and not ".git" in splitted
            and not splitted[-2] == package_name
        ):
            return root


def from_share_to_src(package_name: str, subfolders: List[str]):
    src_path = os.path.join(get_src_path(package_name), *subfolders)
    share_path = os.path.join(get_package_share_directory(package_name), *subfolders)
    shutil.rmtree(src_path)
    shutil.copytree(share_path, src_path)


def from_src_to_share(package_name: str, subfolders: List[str]):
    src_path = os.path.join(get_src_path(package_name), *subfolders)
    share_path = os.path.join(get_package_share_directory(package_name), *subfolders)
    shutil.rmtree(share_path)
    shutil.copytree(src_path, share_path)


def load_dataclass_from_yaml(yaml_data, cls):
    if isinstance(yaml_data, dict):
        kwargs = {}
        for key, value in yaml_data.items():
            if key in cls.__annotations__:
                field_type = cls.__annotations__[key]
                if is_dataclass(field_type):
                    nested_instance = load_dataclass_from_yaml(value, field_type)
                    kwargs[key] = nested_instance
                elif isinstance(value, dict):
                    nested_kwargs = {}
                    field_type = inspect.currentframe().f_back.f_globals[
                        str(cls.__annotations__[key])
                        .split(",")[-1][:-1]
                        .strip()
                        .split(".")[-1]
                    ]
                    for k, v in value.items():
                        nested_kwargs[k] = load_dataclass_from_yaml(v, field_type)
                    kwargs[key] = nested_kwargs
                else:
                    kwargs[key] = value
        return cls(**kwargs)
    else:
        return yaml_data


def remove_none_values(d: Dict[str, Any]) -> Dict[str, Any]:
    """Removes None values from dictionary

    Args:
        d (Dict[str, Any]): dictionary with None values

    Returns:
        Dict[str, Any]: dictionary without None values
    """
    if isinstance(d, dict):
        # Create a copy of the dictionary to avoid modifying it while iterating
        d_copy = d.copy()
        for key, value in d_copy.items():
            # Recursively remove None values in nested dictionaries
            if value is None:
                d.pop(key)
            elif isinstance(value, dict):
                remove_none_values(value)
    return d
