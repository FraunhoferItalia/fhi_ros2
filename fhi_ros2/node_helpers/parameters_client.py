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
from typing import Any, Dict, List, Tuple
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rcl_interfaces.msg import ParameterValue

from ros2param.api import get_value


class ParametersClient:
    def __init__(
        self,
        node: Node,
        parameter_server_name: str,
        callback_group=None,
    ) -> None:
        self.node = node
        self.parameter_server_name = parameter_server_name
        callback_group = (
            MutuallyExclusiveCallbackGroup()
            if callback_group is None
            else callback_group
        )
        self.client = node.create_client(
            GetParameters,
            f"{parameter_server_name}/get_parameters",
            callback_group=callback_group,
        )
        self.list_client = node.create_client(
            ListParameters,
            f"{parameter_server_name}/list_parameters",
            callback_group=callback_group,
        )
        self.set_client = node.create_client(
            SetParameters,
            f"{parameter_server_name}/set_parameters",
            callback_group=callback_group,
        )

    def get_parameters(
        self, parameter_names: List[str], timeout=None
    ) -> List[ParameterValue]:
        request = GetParameters.Request()
        request.names = parameter_names
        self.node.get_logger().debug(f"requesting: {parameter_names}")

        if not self.client.wait_for_service(timeout):
            raise TimeoutError(
                f"Unable to get parameters `{parameter_names}` from node `{self.parameter_server_name}`: {self.client.srv_name} not available"
            )

        self.result = self.client.call(request)
        self.node.get_logger().debug(f"received!")
        return self.result.values

    def get_parameters_list(
        self, prefixes: List[str] = [], timeout=None
    ) -> List[ParameterValue]:
        request = ListParameters.Request()
        request.prefixes = prefixes
        self.node.get_logger().debug(f"requesting: {request}")

        if not self.list_client.wait_for_service(timeout):
            raise TimeoutError(
                f"Unable to list parameters of node `{self.parameter_server_name}`: {self.list_client.srv_name} not available"
            )

        self.result = self.list_client.call(request).result
        self.node.get_logger().debug(f"received!")
        return self.result.names

    def set_parameters(
        self, parameters_dict: Dict[str, Any], timeout=None
    ) -> Tuple[bool, str]:
        request = SetParameters.Request()
        for name, value in parameters_dict.items():
            self.node.get_logger().debug(
                f"requesting set of param {name} to value {value}"
            )
            request.parameters.append(
                Parameter(name=name, value=value).to_parameter_msg()
            )

        if not self.set_client.wait_for_service(timeout):
            raise TimeoutError(
                f"Unable to set parameters of node `{self.parameter_server_name}`: {self.set_client.srv_name} not available"
            )

        result = self.set_client.call(request).results
        return [(r.successful, r.reason) for r in result]

    def dump(self, timeout=None) -> Dict[str, Any]:
        parameter_names = self.get_parameters_list(timeout=timeout)
        parameters = self.get_parameters(parameter_names, timeout)
        return {
            parameter_name: get_value(parameter_value=parameter)
            for parameter_name, parameter in zip(parameter_names, parameters)
        }
