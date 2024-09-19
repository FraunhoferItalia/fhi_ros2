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
"""Module for the `PushUseSimTime` action."""

from typing import List

from launch import Action
from launch import Substitution
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext


@expose_action("push_use_sim_time")
@expose_action("push-use-sim-time")
class PushUseSimTime(Action):
    """
    Action that pushes the `use_sim_time` parameter.

    It's automatically popped when used inside a scoped `GroupAction`.
    There's no other way of popping it.
    """

    def __init__(self, **kwargs) -> None:
        """Create a PushUseSimTime action."""
        super().__init__(**kwargs)
        self.__use_sim_time = True

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `PushUseSimTime` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs["use_sim_time"] = parser.parse_substitution(
            entity.get_attr("use_sim_time")
        )
        return cls, kwargs

    @property
    def use_sim_time(self) -> List[Substitution]:
        """Getter for self.__namespace."""
        return self.__use_sim_time

    def execute(self, context: LaunchContext):
        """Execute the action."""
        context.launch_configurations["global_params"] = {"use_sim_time": True}.items()
