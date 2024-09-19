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
from typing import List, Optional
from .moveit_config import MoveItConfigs, MoveItConfigsBuilder
from ..utils import merge_dict


def merge_moveit_controllers(moveit_controllers):
    controller_names = []
    merged_moveit_controllers = {}
    for moveit_controller in moveit_controllers:
        for controller_name in moveit_controller["controller_names"]:
            controller_names.append(controller_name)
            merged_moveit_controllers[controller_name] = moveit_controller[
                controller_name
            ]

    merged_moveit_controllers["controller_names"] = controller_names
    return {"moveit_simple_controller_manager": merged_moveit_controllers}


def merge_trajectory_execution(trajectory_executions):
    merged_moveit_controllers = merge_moveit_controllers(
        [
            trajectory_execution["moveit_simple_controller_manager"]
            for trajectory_execution in trajectory_executions
        ]
    )

    merged_trajectory_execution = trajectory_executions.pop(0)
    for trajectory_execution in trajectory_executions:
        merged_trajectory_execution.update(trajectory_execution)

    merged_trajectory_execution.update(merged_moveit_controllers)

    return merged_trajectory_execution


def merge_planning_pipelines(planning_pipelines):
    merged_planning_pipelines = planning_pipelines.pop(0)
    for planning_pipeline in planning_pipelines:
        merged_planning_pipelines["ompl"].update(planning_pipeline["ompl"])

    return merged_planning_pipelines


class MoveItConfigsMerger:
    moveit_config: MoveItConfigs
    configs: List[MoveItConfigs]

    def __init__(
        self,
        robot_name: str,
        configs: List[MoveItConfigs],
        package_name: Optional[str] = None,
        use_sim_time: bool = False,
    ):
        self.moveit_config = (
            MoveItConfigsBuilder(
                robot_name=robot_name,
                package_name=package_name,
                use_sim_time=use_sim_time,
            )
            .robot_description_semantic()
            .robot_description_kinematics()
            .robot_description_kinematics_str()
            .get_config()
        )

        self.reference_config = configs[0]
        self.configs = configs

    def joint_limits(self):
        self.moveit_config.joint_limits = self.configs[0].joint_limits
        for i in range(1, len(self.configs)):
            joint_limits_field = merge_dict(
                self.moveit_config.joint_limits["robot_description_planning"][
                    "joint_limits"
                ],
                self.configs[i].joint_limits["robot_description_planning"][
                    "joint_limits"
                ],
            )
            self.moveit_config.joint_limits.update(self.configs[i].joint_limits)
            self.moveit_config.joint_limits["robot_description_planning"][
                "joint_limits"
            ] = joint_limits_field

        self.moveit_config.pilz_cartesian_limits = (
            self.reference_config.pilz_cartesian_limits
        )

    def trajectory_execution(self):
        self.moveit_config.trajectory_execution = merge_trajectory_execution(
            [config.trajectory_execution for config in self.configs]
        )

    def planning_pipelines(self):
        self.moveit_config.planning_pipelines = merge_planning_pipelines(
            [config.planning_pipelines for config in self.configs]
        )

    def planning_scene_monitor(self):
        self.moveit_config.planning_scene_monitor = (
            self.reference_config.planning_scene_monitor
        )

    def to_moveit_configs(self):
        self.joint_limits()
        self.trajectory_execution()
        self.planning_pipelines()
        self.planning_scene_monitor()

        return self.moveit_config
