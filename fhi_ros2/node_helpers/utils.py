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
import subprocess


def node_exists(node_name: str):
    # Define your Bash function command
    bash_command = "ros2 node list"

    # Execute the Bash command and capture the output
    process = subprocess.Popen(bash_command, stdout=subprocess.PIPE, shell=True)
    output, _ = process.communicate()

    # Decode the output and split it into lines
    output_lines = output.decode().splitlines()

    # Process each line of the output
    node_names = [line.split("/")[-1] for line in output_lines]
    return node_name in node_names
