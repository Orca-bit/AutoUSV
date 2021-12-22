# Copyright 2021 The Autoware Foundation
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
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


from autoware_auto_planning_msgs.action import PlannerCostmap
from autoware_auto_mapping_msgs.srv import HADMapService

from geometry_msgs.msg import TransformStamped

from ament_index_python import get_package_share_directory

import rclpy
from rclpy.action import ActionClient

from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import tf2_ros

import os
import time
import unittest


class TransformBroadcaster(rclpy.node.Node):
    def __init__(self):
        super().__init__('transform_broadcaster')
        self._broadcaster = tf2_ros.TransformBroadcaster(self)

    def broadcast_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        self._broadcaster.sendTransform(t)


class HADMapServiceMock(rclpy.node.Node):
    def __init__(self):
        super().__init__('HAD_Map_Service')
        self._service_server = self.create_service(
            HADMapService,
            'HAD_Map_Service',
            self.HAD_map_callback)

    def HAD_map_callback(self, request, response):
        response = HADMapService.Response()

        # parking place from AutonomousStuff's map placed in map frame's (0,0,0) position
        response.map.data = [
            22, 0, 0, 0, 0, 0, 0, 0, 115, 101, 114, 105, 97, 108, 105, 122, 97, 116, 105,
            111, 110, 58, 58, 97, 114, 99, 104, 105, 118, 101, 17, 0, 4, 8, 4, 8, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 3, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 153, 31,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 104, 181, 197, 217, 251, 63, 0, 0, 192, 159, 207, 249, 4,
            64, 0, 0, 0, 128, 87, 7, 250, 191, 3, 0, 1, 0, 0, 0, 124, 34, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 52,
            70, 242, 196, 3, 192, 0, 0, 64, 16, 63, 101, 235, 191, 0, 0, 0, 96, 70, 76,
            249, 191, 3, 0, 2, 0, 0, 0, 125, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 169, 64, 32, 232, 191, 0, 0,
            0, 33, 98, 108, 7, 192, 0, 0, 0, 0, 2, 43, 251, 191, 3, 0, 3, 0, 0,
            0, 122, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 232, 44, 197, 169, 11, 64, 0, 0, 192, 126, 244, 154, 225, 63, 0,
            0, 0, 96, 0, 83, 250, 191, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 1, 0, 0, 0, 7, 0, 1, 0, 0, 0, 0, 4, 0, 0, 0, 225,
            61, 254, 255, 255, 255, 255, 255, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 99, 97, 100, 95, 105, 100,
            0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 52, 54, 52, 50, 49, 56,
            8, 0, 0, 0, 0, 0, 0, 0, 100, 114, 105, 118, 97, 98, 108, 101, 4, 0, 0,
            0, 0, 0, 0, 0, 84, 114, 117, 101, 5, 0, 0, 0, 0, 0, 0, 0, 108, 101,
            118, 101, 108, 1, 0, 0, 0, 0, 0, 0, 0, 49, 13, 0, 0, 0, 0, 0, 0,
            0, 112, 97, 114, 107, 105, 110, 103, 95, 115, 112, 111, 116, 115, 14, 0, 0, 0, 0,
            0, 0, 0, 57, 53, 51, 48, 44, 56, 50, 54, 56, 44, 56, 56, 50, 53, 11, 0,
            0, 0, 0, 0, 0, 0, 114, 101, 102, 95, 108, 97, 110, 101, 108, 101, 116, 9, 0,
            0, 0, 0, 0, 0, 0, 54, 52, 51, 52, 44, 54, 52, 52, 49, 7, 0, 0, 0,
            0, 0, 0, 0, 115, 117, 98, 116, 121, 112, 101, 14, 0, 0, 0, 0, 0, 0, 0,
            112, 97, 114, 107, 105, 110, 103, 95, 97, 99, 99, 101, 115, 115, 4, 0, 0, 0, 0,
            0, 0, 0, 116, 121, 112, 101, 4, 0, 0, 0, 0, 0, 0, 0, 97, 114, 101, 97,
            0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0,
            3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 1, 7, 0, 5, 0, 0, 0, 88, 153,
            1, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6,
            0, 0, 0, 0, 0, 0, 0, 99, 97, 100, 95, 105, 100, 6, 0, 0, 0, 0, 0,
            0, 0, 51, 56, 57, 51, 51, 49, 6, 0, 0, 0, 0, 0, 0, 0, 99, 101, 110,
            116, 101, 114, 4, 0, 0, 0, 0, 0, 0, 0, 56, 56, 51, 50, 8, 0, 0, 0,
            0, 0, 0, 0, 100, 114, 105, 118, 97, 98, 108, 101, 4, 0, 0, 0, 0, 0, 0,
            0, 84, 114, 117, 101, 5, 0, 0, 0, 0, 0, 0, 0, 108, 101, 118, 101, 108, 1,
            0, 0, 0, 0, 0, 0, 0, 49, 16, 0, 0, 0, 0, 0, 0, 0, 112, 97, 114,
            107, 105, 110, 103, 95, 97, 99, 99, 101, 115, 115, 101, 115, 4, 0, 0, 0, 0, 0,
            0, 0, 56, 48, 56, 53, 7, 0, 0, 0, 0, 0, 0, 0, 115, 117, 98, 116, 121,
            112, 101, 12, 0, 0, 0, 0, 0, 0, 0, 112, 97, 114, 107, 105, 110, 103, 95, 115,
            112, 111, 116, 4, 0, 0, 0, 0, 0, 0, 0, 116, 121, 112, 101, 4, 0, 0, 0,
            0, 0, 0, 0, 97, 114, 101, 97, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 3, 0, 0, 0, 0, 0, 3, 0, 1, 0, 0, 0, 3, 0, 2, 0, 0, 0,
            3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 13, 0, 1, 0, 0, 0,
            0, 6, 0, 0, 0, 121, 34, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 99, 97, 100, 95, 105,
            100, 6, 0, 0, 0, 0, 0, 0, 0, 51, 56, 57, 51, 51, 49, 6, 0, 0, 0,
            0, 0, 0, 0, 99, 101, 110, 116, 101, 114, 4, 0, 0, 0, 0, 0, 0, 0, 56,
            56, 51, 50, 8, 0, 0, 0, 0, 0, 0, 0, 100, 114, 105, 118, 97, 98, 108, 101,
            4, 0, 0, 0, 0, 0, 0, 0, 84, 114, 117, 101, 5, 0, 0, 0, 0, 0, 0,
            0, 108, 101, 118, 101, 108, 1, 0, 0, 0, 0, 0, 0, 0, 49, 16, 0, 0, 0,
            0, 0, 0, 0, 112, 97, 114, 107, 105, 110, 103, 95, 97, 99, 99, 101, 115, 115, 101,
            115, 4, 0, 0, 0, 0, 0, 0, 0, 56, 48, 56, 53, 7, 0, 0, 0, 0, 0,
            0, 0, 115, 117, 98, 116, 121, 112, 101, 12, 0, 0, 0, 0, 0, 0, 0, 112, 97,
            114, 107, 105, 110, 103, 95, 115, 112, 111, 116, 4, 0, 0, 0, 0, 0, 0, 0, 116,
            121, 112, 101, 12, 0, 0, 0, 0, 0, 0, 0, 109, 117, 108, 116, 105, 112, 111, 108,
            121, 103, 111, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 7, 0, 5, 0, 0, 0, 1, 7, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            194, 157, 1, 0, 0, 0, 0, 0]

        return response


class GenerateCostmapClientMock(rclpy.node.Node):
    def __init__(self):
        super().__init__('generate_costmap_client_mock')
        self._action_client = ActionClient(
            self,
            PlannerCostmap,
            'generate_costmap')
        self._send_goal_future = None
        self._get_result_future = None
        self._result = None

    def send_goal(self, goal):
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        rclpy.spin_until_future_complete(self, self._get_result_future)

    def get_result_callback(self, future):
        self._result = future.result().result


def generate_test_description():
    costmap_generator_node = launch_ros.actions.Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        parameters=[os.path.join(
            get_package_share_directory('costmap_generator_nodes'),
            'param/test.param.yaml'
        )],
        remappings=[
            ('~/client/HAD_Map_Service', '/HAD_Map_Service')
        ]
    )

    context = {'costmap_generator_node': costmap_generator_node}

    return LaunchDescription([
        # costmap generator - tested node
        costmap_generator_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


class TestBasicUsage(unittest.TestCase):
    def test_basic_case_works(self, costmap_generator_node):
        rclpy.init()

        had_map_service = HADMapServiceMock()

        generate_costmap_client = GenerateCostmapClientMock()

        tf_broadcaster = TransformBroadcaster()

        # costmap generator requires "base_link" -> "map" transform to initialize
        for i in range(0, 20):
            tf_broadcaster.broadcast_transform()
            time.sleep(0.1)

        goal = PlannerCostmap.Goal()
        goal.route.header.stamp = generate_costmap_client.get_clock().now().to_msg()
        goal.route.header.frame_id = "map"
        # short route because mocked lanelet is very small
        goal.route.start_pose.position.x = 10.0
        goal.route.start_pose.position.y = 12.5
        goal.route.goal_pose.position.x = 11.0
        goal.route.goal_pose.position.y = 12.5

        generate_costmap_client.send_goal(goal)

        while not generate_costmap_client._result:
            rclpy.spin_once(had_map_service)
            time.sleep(0.1)

        result = generate_costmap_client._result

        # configured values
        self.assertEqual(result.costmap.header.frame_id, "map")
        self.assertTrue(abs(result.costmap.info.resolution - 0.2) < 0.001)

        # non-zero dimensions and non-empty data
        self.assertTrue(result.costmap.info.width > 0)
        self.assertTrue(result.costmap.info.height > 0)
        self.assertTrue(len(result.costmap.data) > 0)

        # trimmed size - it means smaller than configured
        self.assertTrue(result.costmap.info.width < 70)
        self.assertTrue(result.costmap.info.height < 70)

        return
