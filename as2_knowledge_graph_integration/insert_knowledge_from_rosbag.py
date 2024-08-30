#!/bin/python3
# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import as2_knowledge_graph_integration.utils as utils
import as2_knowledge_graph_integration.utils_for_drones as utils_for_drones
import as2_knowledge_graph_integration.parameters as parameters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from as2_msgs.msg import PlatformInfo
from as2_knowledge_graph_msgs.srv import CreateNode, CreateEdge
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import rosbag2_py
from tf2_msgs.msg import TFMessage


class InsertKnowledgeService(RclNode):

    def __init__(self):
        super().__init__('insert_knowledge_node_from_rosbag')

        # self.reader = rosbag2_py.SequentialReader()
        # storage_options = rosbag2_py.StorageOptions(
        #     uri='/home/carmendrpr/ros2_ws/src/rosbag2_2024_07_29-cf_gates', storage_id='sqlite3')
        # converter_options = rosbag2_py.ConverterOptions(
        #     input_serialization_format='cdr', output_serialization_format='cdr')
        # self.reader.open(storage_options, converter_options)
        # self.topic_of_interest = '/tf'

        """Timers"""
        self.timer_nodes = self.create_timer(2.0, self.generate_nodes)
        self.timer_edges = self.create_timer(3, self.generate_edges)

        """Clients"""
        self.cli_create_node = self.create_client(CreateNode, '/create_node')
        self.cli_create_edge = self.create_client(CreateEdge, '/create_edge')
        self.cli_remove_edge = self.create_client(CreateEdge, '/remove_edge')
        self.cli_remove_node = self.create_client(CreateNode, '/remove_node')

        # Checking if the services are available
        while not self.cli_create_node.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_node is not available, waiting again...')

        while not self.cli_create_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_edge is not available, waiting again...')

        while not self.cli_remove_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service remove_edge is not available, waiting again...')

        while not self.cli_remove_node.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service remove_node is not available, waiting again...')

        """Subscription"""
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.subscribe_pose, qos_profile_sensor_data)
        self.batery_subscription = self.create_subscription(
            BatteryState, 'sensor_measurements/battery', self.subscribe_baterry, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PlatformInfo, 'platform/info', self.subscribe_status, qos_profile_sensor_data)
        # self.subscription = self.create_subscription(
        #     TFMessage, 'tf', self.subscribe_gates, qos_profile_sensor_data)
        """Dictionary the futures"""
        self.responses = {}
        self.request = {'dron': CreateNode.Request(), 'pose': CreateNode.Request(), 'battery': CreateNode.Request(), 'status': CreateNode.Request(
        ), 'status_rm': CreateNode.Request(), 'gate': CreateNode.Request()}
        self.request['status_rm'].node = utils.node_format(
            'Status', 'Disarmed', 1)
        self.request_edge = {'status_edge': CreateEdge.Request(
        ), 'battery_edge': CreateEdge.Request(), 'gate_edge': CreateEdge.Request(), 'gate_edge_rm': CreateEdge.Request()}

        self.request['gate'].node = parameters.gate_node

    def subscribe_pose(self, msg: PoseStamped) -> None:
        self.drone_node = utils.node_format_with_prop(
            'Dron', self.get_namespace(), utils_for_drones.pos_prop_from_pose(msg.pose), priority=1)

    def subscribe_baterry(self, msg: BatteryState) -> None:
        self.drone_baterry = utils.node_format_with_prop(
            'Battery', 'Battery', utils_for_drones.charge_prop_from_battery(msg), priority=1)

    def subscribe_status(self, msg: PlatformInfo) -> None:
        self.drone_status = utils.node_format(
            'Status', utils_for_drones.status_from_platform(msg.status.state), priority=1)

    # def subscribe_gates(self, msg: TFMessage) -> None:
    #     while self.reader.has_next():
    #         topic, data, timestamp = self.reader.read_next()
    #         if topic == self.topic_of_interest:
    #             print(f"Topic: {str(topic)}, Data: {str(data)}")

    def static_nodes(self):
        """
        Request for the static nodes
        """
        self.request['dron'].node = self.drone_node

        """
        Responses for the static nodes
        """
        self.responses['namespace'] = self.cli_create_node.call_async(self.request['dron'])
        rclpy.spin_until_future_complete(self, self.responses['namespace'])

    def generate_nodes(self):
        """
        Request for the nodes
        """

        self.request['battery'].node = self.drone_baterry

        self.request['status'].node = self.drone_status

        self.request_edge['status_edge'].edge = utils.edge_format(
            'is ', self.drone_node.node_name, self.drone_status.node_name)

        if self.request['status_rm'].node != self.drone_status:
            self.cli_remove_node.call_async(self.request['status_rm'])
            self.request['status_rm'].node = self.drone_status

        """
        Responses
        """

        self.responses['battery'] = self.cli_create_node.call_async(self.request['battery'])
        self.responses['status'] = self.cli_create_node.call_async(self.request['status'])
        self.responses['status_edge'] = self.cli_create_edge.call_async(
            self.request_edge['status_edge'])
        self.responses['gate'] = self.cli_create_node.call_async(self.request['gate'])

    def generate_edges(self):
        """
        Request for the edges
        """

        self.request_edge['gate_edge'].edge = utils.edge_format(
            'near ', self.drone_node.node_name, self.request['gate'].node.node_name)
        self.request_edge['gate_edge_rm'].edge = utils.edge_format(
            'apart ', self.drone_node.node_name, self.request['gate'].node.node_name)
        self.request_edge['battery_edge'].edge = utils_for_drones.batery_edge_req(
            self.drone_baterry.properties[1].value.float_value, self.drone_node.node_name, self.drone_baterry.node_name)

        """
        Responses for the edges
        """

        self.responses['battery_edge'] = self.cli_create_edge.call_async(
            self.request_edge['battery_edge'])

        if utils_for_drones.check_distances(self.drone_node, self.request['gate'].node):
            self.responses['gate_edge'] = self.cli_create_edge.call_async(
                self.request_edge['gate_edge'])
            self.responses['gate_edge_rm'] = self.cli_remove_edge.call_async(
                self.request_edge['gate_edge_rm'])

        else:
            self.responses['gate_edge'] = self.cli_remove_edge.call_async(
                self.request_edge['gate_edge'])
            self.responses['gate_edge_rm'] = self.cli_create_edge.call_async(
                self.request_edge['gate_edge_rm'])


def main():
    rclpy.init()
    service_node = InsertKnowledgeService()
    import time

    while rclpy.ok():

        rclpy.spin_once(service_node)
        service_node.static_nodes()
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
