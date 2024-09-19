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
from os.path import join as join_path
import yaml
from yaml.scanner import ScannerError
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ..utils import get_src_path

# the file_path has the following structure for conventions:
# <package_name>/<subfolder1>/<filename>.<file_extension>


def get_absolute_file_path(path: str):
    if not path or path.startswith("/"):
        return path

    splitted = path.split("/")
    return join_path(
        get_package_share_directory(splitted[0]),
        *splitted[1:],
    )


def get_absolute_src_file_path(file_path):
    file_path = file_path.split("/")
    package_name = file_path.pop(0)
    return join_path(get_src_path(package_name), *file_path)


def load_yaml(file_path, pkg_relative=True):
    with open(
        get_absolute_file_path(file_path) if pkg_relative else file_path, "r"
    ) as file:
        return yaml.safe_load(file)


def save_yaml(file_path, content):
    with open(get_absolute_file_path(file_path), "w") as file:
        yaml.dump(content, file)


def get_launcher(package_name, launch_name, subfolders=["launch"]):
    return PythonLaunchDescriptionSource(
        [
            join_path(
                get_package_share_directory(package_name),
                *subfolders,
                f"{launch_name}.launch.py",
            )
        ]
    )


def load_malformed_yaml(yaml_str: str, debug: bool = False):
    """
    Loads malformed yaml by removeing non dict lines.
    """
    while True:
        try:
            return yaml.safe_load(yaml_str)
        except ScannerError as e:
            yaml_lines = yaml_str.split("\n")
            malformed_line = yaml_lines.pop(e.context_mark.line)
            if debug:
                print(f"Found malformed line: {malformed_line}")
            yaml_str = "\n".join(yaml_lines)


def adapt_controller_yaml_to_namespace(file_path, namespace, suffix_string="temp"):
    controller_yaml = load_yaml(file_path)
    file_path = file_path.split(".")[0] + f"_{suffix_string}.yaml"
    save_yaml(
        file_path, controller_yaml if namespace == "" else {namespace: controller_yaml}
    )


def bool_to_str(bool):
    return "true" if bool else "false"


def get_action_remappings(old: str, new: str):
    return [
        (f"{old}/_action/feedback", f"{new}/_action/feedback"),
        (f"{old}/_action/status", f"{new}/_action/status"),
        (f"{old}/_action/cancel_goal", f"{new}/_action/cancel_goal"),
        (f"{old}/_action/get_result", f"{new}/_action/get_result"),
        (f"{old}/_action/send_goal", f"{new}/_action/send_goal"),
    ]
