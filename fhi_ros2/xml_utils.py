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
import copy
from typing import List
from xml.dom import minidom
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

XACRO_NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO_NS)


def get_subtree(tree: Element, subtree_name: str) -> Element:
    for child in tree:
        if child.tag == subtree_name:
            return child


def recurse_subtree(tree: Element, subtree_names: List[str]) -> Element:
    next_element = copy.deepcopy(tree)
    for i in range(len(subtree_names)):
        next_element = get_subtree(next_element, subtree_names[i])
        if next_element is None:
            break

    return next_element


def prettify_xml(elem: Element):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, "utf-8", method="xml")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml("\t", "\n")


def remove_comments_from_xml(file_path: str):
    with open(file_path, mode="r", encoding="utf-8") as f:
        file_data = f.read()

    with open(file_path, mode="w", encoding="utf-8") as out_file:
        ET.canonicalize(file_data, out=out_file)
