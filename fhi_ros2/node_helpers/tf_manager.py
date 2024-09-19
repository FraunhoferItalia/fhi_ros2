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
from __future__ import annotations
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform, TransformStamped, Pose
from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration
from rclpy.time import Time


class Transformation:
    def __init__(self, matrix=np.eye(4)):
        self.matrix = matrix

    @property
    def quaternion(self):
        return R.from_matrix(self.matrix[:3, :3]).as_quat()

    @property
    def translation(self):
        return self.matrix[0:3, 3]

    def from_transformation_msg(msg: Transform) -> Transformation:
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        ).as_matrix()
        matrix[:3, 3] = np.array(
            [msg.translation.x, msg.translation.y, msg.translation.z]
        )
        return Transformation(matrix)

    def from_odometry_msg(msg: Odometry) -> Transformation:
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        ).as_matrix()
        matrix[:3, 3] = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        return Transformation(matrix)

    def from_pose_msg(msg: Pose) -> Transformation:
        matrix = np.eye(4)
        matrix[:3, :3] = R.from_quat(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        ).as_matrix()
        matrix[:3, 3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        return Transformation(matrix)

    def from_xyzrpy(xyzrpy: List[float]) -> Transformation:
        matrix = np.eye(4)
        matrix[:3, 3] = xyzrpy[:3]
        matrix[:3, :3] = R.from_euler("xyz", xyzrpy[3:]).as_matrix()
        return Transformation(matrix)

    def to_xyzrpy(self, degrees=False, rotation_sequence="xyz") -> List[float]:
        return list(self.translation) + list(
            R.from_matrix(self.matrix[:3, :3]).as_euler(
                rotation_sequence, degrees=degrees
            )
        )

    def to_pose_msg(self) -> Pose:
        pose = Pose()
        translation = self.translation
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        quaternion = self.quaternion
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def to_transformation_msg(self) -> Transform:
        transform = Transform()
        translation = self.translation
        transform.translation.x = translation[0]
        transform.translation.y = translation[1]
        transform.translation.z = translation[2]
        quaternion = self.quaternion
        transform.rotation.x = quaternion[0]
        transform.rotation.y = quaternion[1]
        transform.rotation.z = quaternion[2]
        transform.rotation.w = quaternion[3]
        return transform

    def to_pose_msg(self) -> Pose:
        pose = Pose()
        translation = self.translation
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        quaternion = self.quaternion
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def __mul__(self, other: Transformation) -> Transformation:
        return Transformation(self.matrix.dot(other.matrix))

    def inv(self):
        return Transformation(np.linalg.inv(self.matrix))


# Credits https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py#L43
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def average_quaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4, 4))

    for i in range(0, M):
        q = Q[i, :]
        # multiply q with its transposed version q' and add A
        A = np.outer(q, q) + A

    # scale
    A = (1.0 / M) * A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0])


def average_transformations(transformations: List[Transformation]) -> Transformation:
    def scalar_last_to_scalar_first(quaternion):
        return [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

    def scalar_first_to_scalar_last(quaternion):
        return [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]

    quaternions = [scalar_last_to_scalar_first(t.quaternion) for t in transformations]
    translations = [t.translation for t in transformations]
    averaged_quaternion = scalar_first_to_scalar_last(
        average_quaternions(np.vstack(quaternions))
    )
    averaged_translation = np.mean(translations, axis=0)

    matrix = np.eye(4)
    matrix[0:3, :3] = R.from_quat(averaged_quaternion).as_matrix()
    matrix[0:3, 3] = averaged_translation

    return Transformation(matrix)


class TFManager:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_broadcaster = TransformBroadcaster(node)

    def get_transformation(
        self, to_frame_rel: str, from_frame_rel: str, time_tolerance: float = 0.0
    ):
        transform_stamped = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time(),
        )
        msg_time = Time.from_msg(transform_stamped.header.stamp)
        if time_tolerance != 0 and (self.node.get_clock().now() - msg_time) > Duration(
            seconds=time_tolerance
        ):
            raise TransformException(
                "Transform timeout!\n"
                f"Time: {self.node.get_clock().now().nanoseconds / 1e9}\n"
                f"Msg Time: {msg_time.nanoseconds / 1e9}\n"
                f"Delta time: {(self.node.get_clock().now() - msg_time).nanoseconds / 1e9}\n"
            )

        return Transformation.from_transformation_msg(transform_stamped.transform)

    def publish(self, transformation: TransformStamped):
        self.tf_broadcaster.sendTransform(transformation)
