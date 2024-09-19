#!/usr/bin/python3
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
from enum import Enum, auto
import time
import rclpy
from typing import Dict, Tuple, Union
from rclpy.node import Node, Client
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ClientType(Enum):
    SERVICE = auto()
    ACTION = auto()


class ClientsHandler:
    action_clients: Dict[str, ActionClient]
    service_clients: Dict[str, Client]
    service_info: Dict
    action_info: Dict

    def __init__(
        self,
        node: Node,
        action_info: Dict,
        service_info: Dict,
        spin: bool = True,
        init_clients_timeout=3.0,
    ):
        self.node = node
        self.action_clients = {}
        self.service_clients = {}
        self.service_info = service_info
        self.action_info = action_info
        self.spin = spin

        self.init_clients(init_clients_timeout)

    def init_clients(self, timeout):
        for name, type in self.action_info.items():
            client = ActionClient(
                self.node, type, name, callback_group=MutuallyExclusiveCallbackGroup()
            )
            if not client.wait_for_server(timeout_sec=timeout):
                self.node.get_logger().warn(f"Action server {name} not available.")
            self.action_clients[name] = client

        for name, type in self.service_info.items():
            client = self.node.create_client(
                type, name, callback_group=MutuallyExclusiveCallbackGroup()
            )
            if not client.wait_for_service(timeout_sec=timeout):
                self.node.get_logger().warn(f"Service server {name} not available.")
            self.service_clients[name] = client

    def spin_node(self):
        if self.spin:
            rclpy.spin_once(self.node)
        else:
            time.sleep(0.01)

    def get_client(
        self, request, name: str = None
    ) -> Tuple[ClientType, Union[Client, ActionClient]]:
        request_type = str(type(request)).split(".")[-1].split("_")[0]
        try:
            if name is None:
                # .split("_") used to take away Metaclass_MyAction from the class name
                service_name = {
                    v.__class__.__name__.split("_")[-1]: k
                    for k, v in self.service_info.items()
                }[request_type]
            else:
                service_name = name
            return ClientType.SERVICE, self.service_clients[service_name]
        except KeyError:
            if name is None:
                # .split("_") used to take away Metaclass_MyAction from the class name
                action_name = {
                    v.__class__.__name__.split("_")[-1]: k
                    for k, v in self.action_info.items()
                }[request_type]
            else:
                action_name = name
            return ClientType.ACTION, self.action_clients[action_name]

    def call_sync(self, request, name: str = None, timeout: float = 0.0):
        client_type, client = self.get_client(request, name)

        if client_type == ClientType.SERVICE:
            if not client.service_is_ready():
                return False
            future = client.call_async(request)
            try:
                return self.wait_future(future, timeout)
            except TimeoutError:
                return False
        elif client_type == ClientType.ACTION:
            if not client.server_is_ready():
                return False
            future = client.send_goal_async(request)

            try:
                goal_handle = self.wait_future(future, timeout)
            except TimeoutError:
                self.node.get_logger().error("Timeout exceeded")
                return False

            if not goal_handle.accepted:
                self.node.get_logger().error("Goal rejected")
                return False

            result_future = goal_handle.get_result_async()

            while not result_future.done():
                self.spin_node()
            return result_future.result().result

    def wait_future(self, future, timeout: float = 0.0):
        start_time = time.time()
        while not future.done():
            self.spin_node()
            if timeout != 0.0 and time.time() - start_time > timeout:
                raise TimeoutError()
        return future.result()

    def call_async(self, request, name: str = None):
        client_type, client = self.get_client(request, name)

        if client_type == ClientType.SERVICE:
            if not client.service_is_ready():
                return False
            return client.call_async(request)
        elif client_type == ClientType.ACTION:
            if not client.server_is_ready():
                return False
            future = client.send_goal_async(request)
            return future
