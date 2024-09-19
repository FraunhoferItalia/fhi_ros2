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
import json
import math
from typing import Any, Dict, List, Tuple
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

import yaml
import xacro
from launch import LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils import MoveItConfigs
from .utils import get_absolute_file_path, get_launcher, bool_to_str


def get_robot_description(
    robot_description_file_path: str, mappings: Dict[str, str] = {}
):
    robot_description_config = xacro.process_file(
        get_absolute_file_path(robot_description_file_path),
        mappings=mappings,
    )
    return robot_description_config.toxml()


def add_controller_data_for_remote_gazebo(
    robot_description: str,
    namespace: str = "",
):
    def get_subtree(tree: Element, subtree_name: str) -> Element:
        for child in tree:
            if child.tag == subtree_name:
                return child

    ## Find `libgazebo_ros2_control` gazebo plugin tag
    root = ET.fromstring(robot_description)
    gazebo_tags = [c for c in root if c.tag == "gazebo"]
    for g in gazebo_tags:
        for c in g:
            if c.tag == "plugin" and "filename" in c.keys():
                if c.attrib["filename"] == "libgazebo_ros2_control.so":
                    controller_manager_config_file_path = get_subtree(
                        c, "parameters"
                    ).text
                    controllers_data = ET.Element("controllers_data")
                    with open(controller_manager_config_file_path, "r") as f:
                        controller_manager_config_file_content = f.read()
                    if namespace and not namespace == "/":
                        c.attrib["name"] = namespace + "_gazebo_ros2_control"
                        # Adjust/Adds <ros><namespace> tag
                        ros_et = get_subtree(c, "ros")
                        if ros_et is None:
                            ros_et = ET.Element("ros")
                            c.append(ros_et)

                        namespace_et = get_subtree(ros_et, "namespace")
                        if namespace_et is None:
                            namespace_et = ET.Element("namespace")

                        ros_et.append(namespace_et)
                        namespace_et.text = namespace
                        controller_manager_config_file_content = (
                            controller_manager_config_file_content.replace(
                                "/**", namespace
                            )
                        )
                    controllers_data.text = controller_manager_config_file_content
                    c.append(controllers_data)

    # Remove world link, reference and involved joints
    for c in root:
        if (
            c.tag == "gazebo"
            and "reference" in c.keys()
            and c.attrib["reference"] == "world"
        ):
            root.remove(c)
        if c.tag == "link" and "name" in c.keys() and c.attrib["name"] == "world":
            root.remove(c)
        if c.tag == "joint" and get_subtree(c, "parent").attrib["link"] == "world":
            root.remove(c)
    return ET.tostring(root, "utf-8", method="xml").decode("utf-8")


def get_robot_state_publisher(
    robot_description: str,
    use_sim_time: bool = None,
    remappings: List[Tuple[str, str]] = None,
    namespace: str = "",
):
    parameters = [{"robot_description": robot_description}]
    if not use_sim_time is None:
        parameters += [{"use_sim_time": use_sim_time}]
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
    )


def get_gazebo_launchers(world_file_path: str, start_paused: bool):
    gzserver = IncludeLaunchDescription(
        get_launcher("gazebo_ros", "gzserver"),
        launch_arguments={
            "world": get_absolute_file_path(world_file_path),
            "verbose": "true",
            "pause": str(start_paused),
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        get_launcher("gazebo_ros", "gzclient"),
    )

    return [gzserver, gzclient]


def get_robot_spawner(
    entity_name: str,
    reference_frame: str = "world",
    xyzrpy: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
):
    return Node(
        name="spawner",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            f"{entity_name}",
            "-reference_frame",
            f"{reference_frame}",
            "-x",
            f"{xyzrpy[0]}",
            "-y",
            f"{xyzrpy[1]}",
            "-z",
            f"{xyzrpy[2]}",
            "-R",
            f"{xyzrpy[3]}",
            "-P",
            f"{xyzrpy[4]}",
            "-Y",
            f"{xyzrpy[5]}",
        ],
    )


def get_moveit_config(
    robot_name: str,
    moveit_pkg_name: str,
    robot_description_mappings: Dict[str, str] = {},
    robot_description_semantic_mappings: Dict[str, str] = {},
    default_planning_pipeline: str = "pilz_industrial_motion_planner",
) -> MoveItConfigs:
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_pkg_name)
        .robot_description(mappings=robot_description_mappings)
        .robot_description_semantic(mappings=robot_description_semantic_mappings)
        .robot_description_kinematics()
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline=default_planning_pipeline,
        )
        .to_moveit_configs()
    )
    return moveit_config


def get_move_group_node(
    moveit_config: MoveItConfigs,
    use_sim_time: bool = False,
):
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )


def get_rviz(
    rviz_config_file: str = "",
    moveit_config: MoveItConfigs = None,
    use_sim_time: bool = False,
):
    parameters = [{"use_sim_time": use_sim_time}]
    if not moveit_config is None:
        parameters += [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", get_absolute_file_path(rviz_config_file)],
        parameters=parameters,
    )


def get_available_controller_names(controller_manager_config_file_path: str):
    with open(controller_manager_config_file_path, "r") as f:
        controller_manager_config = yaml.safe_load(f)

    controller_manager_config = (
        controller_manager_config["/**"]
        if "/**" in controller_manager_config.keys()
        else controller_manager_config
    )
    controller_names = []
    for key, item in controller_manager_config["controller_manager"][
        "ros__parameters"
    ].items():
        if not isinstance(item, dict):
            continue
        try:
            item["type"]
            controller_names.append(key)
        except KeyError:
            pass

    return controller_names


def get_controller_manager(
    robot_description: str, controller_manager_config_file_path: str
):
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            get_absolute_file_path(controller_manager_config_file_path),
        ],
        output="both",
        # prefix=['xterm -e gdb -ex run --args'],
    )


def get_ros2_controllers_spawners(
    controller_manager_config_file_path: str,
    activate_controller_names: List[str],
    timeout: int = 20,
):
    controllers = {}
    for controller_name in get_available_controller_names(
        get_absolute_file_path(controller_manager_config_file_path)
    ):
        controller_args = [
            controller_name,
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            str(int(timeout)),
        ]
        if not controller_name in activate_controller_names:
            controller_args.append("--inactive")

        controllers[controller_name + "_controller_spawner"] = Node(
            package="controller_manager",
            executable="spawner",
            arguments=controller_args,
            output="screen",
        )

    return controllers


def get_static_transform_publisher(
    frame_id: str,
    child_frame_id: str,
    xyzrpy: List[float],
    namespace: str = "",
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace=namespace,
        output="log",
        arguments=[
            "--x",
            str(xyzrpy[0]),
            "--y",
            str(xyzrpy[1]),
            "--z",
            str(xyzrpy[2]),
            "--roll",
            str(xyzrpy[3]),
            "--pitch",
            str(xyzrpy[4]),
            "--yaw",
            str(xyzrpy[5]),
            "--frame-id",
            frame_id,
            "--child-frame-id",
            child_frame_id,
        ],
    )


def get_tf_manager_attacher(
    parent_frame_id: str,
    child_frame_id: str,
    xyzrpy: List[str] = [0.0] * 6,
    tf_manager_node_name: str = "/tf_manager",
):
    def euler_to_quat(rpy):
        cy, sy = math.cos(rpy[2] * 0.5), math.sin(rpy[2] * 0.5)
        cp, sp = math.cos(rpy[1] * 0.5), math.sin(rpy[1] * 0.5)
        cr, sr = math.cos(rpy[0] * 0.5), math.sin(rpy[0] * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]

    q = euler_to_quat(xyzrpy[3:])
    msg = {
        "child_frame_id": child_frame_id,
        "parent_frame_id": parent_frame_id,
        "transform": {
            "translation": {"x": xyzrpy[0], "y": xyzrpy[1], "z": xyzrpy[2]},
            "rotation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
        },
        "disable_link": "false",
    }
    return ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                f"{tf_manager_node_name}/set_transformation ",
                "tf_manager_msgs/srv/SetTransformation ",
                '"' + json.dumps(msg) + '"',
            ]
        ],
        output="both",
        shell=True,
    )


class Options:
    def parse(self, context: LaunchContext):
        pass

    def declare(self):
        pass


@dataclass
class WorldOptions(Options):
    world_file: str = ""
    paused: bool = False

    def parse(self, context: LaunchContext):
        self.world_file = LaunchConfiguration("world_file").perform(context)
        self.paused = LaunchConfiguration("paused").perform(context) == "true"

    def declare(self):
        return [
            DeclareLaunchArgument(
                "world_file",
                default_value=self.world_file,
                description="Path to world file",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value=bool_to_str(self.paused),
                description="Allow to select if simulation starts paused",
                choices=["true", "false"],
            ),
        ]


@dataclass
class RobotOptions(Options):
    robot_name: str
    robot_description_path: str
    moveit_config_pkg: str
    controllers_file_path: str
    robot_description_mappings: Dict[str, str] = field(default_factory=dict)
    robot_description_semantic_mappings: Dict[str, str] = field(default_factory=dict)
    active_controllers: List[str] = field(default_factory=list)

    def parse(self, context: LaunchContext):
        # self.robot_description_path = LaunchConfiguration(
        #     "robot_description_path"
        # ).perform(context)
        # self.moveit_config_pkg = LaunchConfiguration("moveit_config_pkg").perform(
        #     context
        # )
        return

    def declare(self):
        return [
            # DeclareLaunchArgument(
            #     "robot_description_path",
            #     default_value=self.robot_description_path,
            #     description="Path to robot description file",
            # ),
            # DeclareLaunchArgument(
            #     "moveit_config_pkg",
            #     default_value=self.moveit_config_pkg,
            #     description="Moveit config package name",
            # ),
        ]

    def set_description_mappings(self, mappings: Dict[str, Any]):
        self.robot_description_mappings.update(mappings)
        self.robot_description_semantic_mappings.update(mappings)


@dataclass
class LaunchOptions(Options):
    namespace: str = ""
    sim: bool = True
    remote_gazebo: bool = False
    moveit: bool = True
    rviz: bool = True
    rviz_file: str = ""

    def parse(self, context: LaunchContext):
        self.namespace = LaunchConfiguration("namespace").perform(context)
        self.sim = LaunchConfiguration("sim").perform(context) == "true"
        self.remote_gazebo = (
            LaunchConfiguration("remote_gazebo").perform(context) == "true"
        )
        self.moveit = LaunchConfiguration("moveit").perform(context) == "true"
        self.rviz = LaunchConfiguration("rviz").perform(context) == "true"
        self.rviz_file = LaunchConfiguration("rviz_file").perform(context)

    def declare(self):
        return [
            DeclareLaunchArgument("namespace", default_value=self.namespace),
            DeclareLaunchArgument(
                "moveit",
                default_value=bool_to_str(self.moveit),
                description="Allow to select if MoveIt should be started",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "sim",
                default_value=bool_to_str(self.sim),
                description="Allow to select simulation mode",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "remote_gazebo",
                default_value=bool_to_str(self.remote_gazebo),
                description="Allow to select remote_gazebo mode (valid only if sim true)",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value=bool_to_str(self.rviz),
                description="Allow to select if visualization should be started",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value=self.rviz_file,
                description="Path to rviz configuration file",
            ),
        ]


@dataclass
class BringupOptions(Options):
    world_options: WorldOptions
    robot_options: RobotOptions
    launch_options: LaunchOptions
    __options: List[LaunchOptions] = field(default_factory=list)

    def __post_init__(self):
        self.__options = [
            self.world_options,
            self.robot_options,
            self.launch_options,
        ]

    def parse(self, context: LaunchContext):
        for o in self.__options:
            o.parse(context)

    def declare(self):
        declared = []
        for o in self.__options:
            declared += o.declare()
        return declared


def get_bringup_launchers(bringup_options: BringupOptions) -> Dict[str, Node]:
    ## DESCRIPTION
    robot_description = get_robot_description(
        bringup_options.robot_options.robot_description_path,
        bringup_options.robot_options.robot_description_mappings,
    )
    if bringup_options.launch_options.remote_gazebo:
        robot_description = add_controller_data_for_remote_gazebo(
            robot_description, bringup_options.launch_options.namespace
        )
    launchers = {
        "robot_state_publisher": get_robot_state_publisher(
            robot_description,
            use_sim_time=bringup_options.launch_options.sim,
        )
    }

    ## STANDOLONE SIMULATION
    if (
        bringup_options.launch_options.sim
        and not bringup_options.launch_options.remote_gazebo
    ):
        gz_launchers = get_gazebo_launchers(
            bringup_options.world_options.world_file,
            bringup_options.world_options.paused,
        )
        launchers["gzserver"] = gz_launchers[0]
        launchers["gzclient"] = gz_launchers[1]
        launchers["robot_gazebo_spawner"] = get_robot_spawner(
            bringup_options.robot_options.robot_name
        )

    ## ROS2 CONTROL
    if not bringup_options.launch_options.sim:
        launchers["controller_manager"] = get_controller_manager(
            robot_description, bringup_options.robot_options.controllers_file_path
        )
    launchers.update(
        get_ros2_controllers_spawners(
            bringup_options.robot_options.controllers_file_path,
            bringup_options.robot_options.active_controllers,
        )
    )

    ## MOVEIT
    moveit_config = get_moveit_config(
        bringup_options.robot_options.robot_name,
        bringup_options.robot_options.moveit_config_pkg,
        bringup_options.robot_options.robot_description_mappings,
        bringup_options.robot_options.robot_description_semantic_mappings,
    )
    if bringup_options.launch_options.moveit:
        launchers["move_group"] = get_move_group_node(
            moveit_config,
            use_sim_time=bringup_options.launch_options.sim,
        )

    ## RVIZ
    if bringup_options.launch_options.rviz:
        launchers["rviz"] = get_rviz(
            bringup_options.launch_options.rviz_file,
            moveit_config=moveit_config,
            use_sim_time=bringup_options.launch_options.sim,
        )

    return launchers
