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
from .params import wait_for_parameters


def get_remote_config(
    node_name: str,
    namespace: str = "",
    timeout: float = float("inf"),
    should_publish: bool = False,
):
    full_node_name = (
        namespace + "/" + node_name if namespace and not namespace == "/" else node_name
    )
    print(f"Trying to get move_group configuration from {full_node_name}")
    moveit_config_dict = wait_for_parameters(full_node_name, timeout)
    for publish_key in [
        "publish_planning_scene",
        "publish_geometry_updates",
        "publish_state_updates",
        "publish_transforms_updates",
    ]:
        try:
            moveit_config_dict[publish_key] = should_publish
        except KeyError:
            pass
    return moveit_config_dict
