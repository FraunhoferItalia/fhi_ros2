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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

import yaml
from ..utils import custom_print, COLORS

from .params import wait_for_parameter
from .utils import load_malformed_yaml

try:
    from moveit_configs_utils import MoveItConfigsBuilder as BaseMoveItConfigsBuilder
    from moveit_configs_utils import MoveItConfigs as BaseMoveItConfigs
    from launch_param_builder.utils import ParameterBuilderFileNotFoundError
except ModuleNotFoundError as e:
    custom_print("MoveItConfigsBuilder not found.", COLORS.FAIL)
    custom_print(
        "MoveIt2 is required to use this module. Make sure it is installed and sourced",
        COLORS.FAIL,
    )
    raise e


@dataclass(slots=True)
class MoveItConfigs(BaseMoveItConfigs):
    robot_description_kinematics_str: Dict = field(default_factory=dict)
    use_sim_time: Dict = field(default_factory=dict)

    def dict(self):
        parameters = self.to_dict()
        parameters.update(self.use_sim_time)
        parameters.update(self.robot_description_kinematics_str)
        return parameters

    def simple_dict(self):
        parameters = {}
        parameters.update(self.use_sim_time)
        parameters.update(self.robot_description_semantic)
        parameters.update(self.robot_description_kinematics)
        parameters.update(self.planning_pipelines)
        return parameters


class MoveItConfigsBuilder(BaseMoveItConfigsBuilder):
    def __init__(
        self,
        robot_name: str,
        robot_description="robot_description",
        package_name: Optional[str] = None,
        use_sim_time: bool = False,
        config_dir_path: str = "config",
    ):
        super().__init__(robot_name, robot_description, package_name)
        self.__config_dir_path = Path(config_dir_path)
        self.__moveit_configs = MoveItConfigs()
        self.__moveit_configs.use_sim_time = {"use_sim_time": use_sim_time}

    def get_config(self):
        return self.__moveit_configs

    def get_remote_config(context, move_group_name="move_group"):

        robot_description_kinematics_str = wait_for_parameter(
            node_name=move_group_name,
            parameter_name="robot_description_kinematics_str",
        )
        robot_description_semantic = wait_for_parameter(
            node_name=move_group_name,
            parameter_name="robot_description_semantic",
        )
        use_sim_time = wait_for_parameter(
            node_name=move_group_name,
            parameter_name="use_sim_time",
        )

        return {
            "use_sim_time": use_sim_time.lower() == "true",
            "robot_description_semantic": robot_description_semantic,
            "robot_description_kinematics": load_malformed_yaml(
                robot_description_kinematics_str
            ),
        }

    def robot_description_kinematics_str(self):
        self.__moveit_configs.robot_description_kinematics_str = {
            "robot_description_kinematics_str": yaml.safe_dump(
                self.__moveit_configs.robot_description_kinematics[
                    "robot_description_kinematics"
                ]
            )
        }
        return self

    def to_moveit_configs(
        self,
        default_planning_pipeline="pilz_industrial_motion_planner",
        srdf_mappings={},
    ) -> MoveItConfigs:
        """Get MoveIt configs from robot_name_moveit_config.
        :return: An MoveItConfigs instance with all parameters loaded.
        """
        if not self.__moveit_configs.robot_description:
            # see https://github.com/ros-planning/moveit2/pull/1806
            try:
                self.robot_description()
            except ParameterBuilderFileNotFoundError as e:
                custom_print(f"{e}", COLORS.WARNING)
                custom_print(
                    "The robot description will be loaded from /robot_description topic",
                    COLORS.WARNING,
                )
        if not self.__moveit_configs.robot_description_semantic:
            self.robot_description_semantic(
                self.__config_dir_path / Path(str(self.__robot_name) + ".srdf"),
                mappings=srdf_mappings,
            )
        if not self.__moveit_configs.robot_description_kinematics:
            self.robot_description_kinematics()
        if not self.__moveit_configs.planning_pipelines:
            self.planning_pipelines(
                default_planning_pipeline=default_planning_pipeline,
                load_all=False,
            )
        if not self.__moveit_configs.trajectory_execution:
            self.trajectory_execution()
        if not self.__moveit_configs.planning_scene_monitor:
            self.planning_scene_monitor()
        if not self.__moveit_configs.sensors_3d:
            self.sensors_3d()
        if not self.__moveit_configs.joint_limits:
            self.joint_limits()
        if not self.__moveit_configs.robot_description_kinematics_str:
            self.robot_description_kinematics_str()
        # TODO(JafarAbdi): We should have a default moveit_cpp.yaml as port of a moveit config package
        # if not self.__moveit_configs.moveit_cpp:
        #     self.moveit_cpp()
        if "pilz_industrial_motion_planner" in self.__moveit_configs.planning_pipelines:
            if not self.__moveit_configs.pilz_cartesian_limits:
                self.pilz_cartesian_limits()
        return self.__moveit_configs
