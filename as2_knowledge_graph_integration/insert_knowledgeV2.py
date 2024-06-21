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
import parameters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from as2_msgs.msg import PlatformInfo
from as2_knowledge_graph_msgs.srv import CreateNode, CreateEdge
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class InsertKnowledgeService(RclNode):

    def __init__(self):
        super().__init__('insert_knowledge_node')

        """Timers"""
        self.timer_nodes = self.create_timer(2.0, self.generate_nodes)
        self.timer_edges = self.create_timer(2.5, self.generate_edges)

        """Subscription"""
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.subscribe_pose, qos_profile_sensor_data)
        self.batery_subscription = self.create_subscription(
            BatteryState, 'sensor_measurements/battery', self.subscribe_baterry, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PlatformInfo, 'platform/info', self.subscribe_status, qos_profile_sensor_data)

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

        """Dictionary the futures"""
        self.responses = {}

    def subscribe_pose(self, msg: PoseStamped) -> None:
        self.drone_node = utils.node_format(
            'Dron', self.get_namespace(), priority=1)

        self.drone_node_pose = utils.node_format_with_prop(
            'Dron', 'Pose', utils_for_drones.pos_prop_from_pose(msg.pose), priority=1)

    def subscribe_baterry(self, msg: BatteryState) -> None:
        self.drone_baterry = utils.node_format_with_prop(
            'Battery', 'Battery', utils_for_drones.charge_prop_from_battery(msg), priority=1)

    def subscribe_status(self, msg: PlatformInfo) -> None:
        self.drone_status = utils.node_format(
            'Status', utils_for_drones.status_from_platform(msg.status.state), priority=1)

    def generate_node_ns(self):
        req_ns = CreateNode.Request()
        req_ns.node = self.drone_node
        self.responses['namespace'] = self.cli_create_node.call_async(req_ns)
        rclpy.spin_until_future_complete(self, self.responses['namespace'])

    def generate_nodes(self):
        """
        Request for the nodes
        """
        req_pose = CreateNode.Request()
        req_pose.node = self.drone_node_pose
        req_battery = CreateNode.Request()
        req_battery.node = self.drone_baterry
        req_status = CreateNode.Request()
        req_status.node = self.drone_status
        req_geozone = CreateNode.Request()
        req_geozone.node = utils_for_drones.node_from_geozone(parameters.geozone)
        req_home = CreateNode.Request()
        req_home.node = parameters.node_home
        req_person = CreateNode.Request()
        req_person.node = parameters.node_person
        """
        Responses
        """

        self.responses['pose'] = self.cli_create_node.call_async(req_pose)
        self.responses['battery'] = self.cli_create_node.call_async(req_battery)
        self.responses['status'] = self.cli_create_node.call_async(req_status)
        self.responses['geozone'] = self.cli_create_node.call_async(req_geozone)
        self.responses['home'] = self.cli_create_node.call_async(req_home)
        self.responses['person'] = self.cli_create_node.call_async(req_person)

    def generate_edges(self):
        """
        Request for the edges
        """
        req_pose_edge = CreateEdge.Request()
        req_pose_edge.edge = utils.edge_format(
            'is at ', self.drone_node.node_name, self.drone_node_pose.node_name)
        req_battery_edge = CreateEdge.Request()
        req_battery_edge.edge = utils_for_drones.batery_edge_req(
            self.drone_baterry.properties[1].value.float_value, self.drone_node.node_name, self.drone_baterry.node_name)
        req_person_edge = CreateEdge.Request()
        req_person_edge_rm = CreateEdge.Request()
        req_person_edge.edge, req_person_edge_rm.edge = utils_for_drones.person_edge_req(
            self.drone_node_pose, parameters.node_person, self.drone_node.node_name)
        req_home_edge = CreateEdge.Request()
        req_home_edge.edge = utils_for_drones.home_edge_req(
            self.drone_node_pose, parameters.node_home, self.drone_node.node_name)
        req_status_edge = CreateEdge.Request()
        req_status_edge.edge = utils.edge_format(
            'is ', self.drone_node.node_name, self.drone_status.node_name)

        """
        Responses for the edges
        """

        self.responses['pose_edge'] = self.cli_create_edge.call_async(req_pose_edge)

        self.responses['battery_edge'] = self.cli_create_edge.call_async(req_battery_edge)

        self.responses['person_edge'] = self.cli_create_edge.call_async(req_person_edge)

        self.responses['person_edge_rm'] = self.cli_remove_edge.call_async(req_person_edge_rm)

        self.responses['home_edge'] = self.cli_create_edge.call_async(req_home_edge)

        self.responses['status_edge'] = self.cli_create_edge.call_async(req_status_edge)

        # print(utils_for_drones.geozone_edge_req(self.drone_node_pose,
        #       utils_for_drones.node_from_geozone, self.drone_node.node_name))


def main():
    rclpy.init()
    service_node = InsertKnowledgeService()
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        service_node.generate_node_ns()

        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
