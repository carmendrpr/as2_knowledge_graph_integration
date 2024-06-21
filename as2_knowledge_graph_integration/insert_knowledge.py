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
        self.timer_1 = self.create_timer(1.0, self.check_futures)
        self.timer_3 = self.create_timer(3.0, self.simulate)

        """Subscription"""
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.read_pose_callback, qos_profile_sensor_data)
        self.batery_subscription = self.create_subscription(
            BatteryState, 'sensor_measurements/battery', self.read_baterry_state_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PlatformInfo, 'platform/info', self.read_status_callback, qos_profile_sensor_data)

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

        # ¿podria crear una lista para integrar todas estas variables?
        """Dictionary the futures"""
        self.responses = {}

        self.responses = {}
        self.dron_node = None
        self.future_resp_node = None
        self.future_resp_battery_node = None
        self.future_resp_battery_edge = None
        self.future_resp_status = None
        self.future_resp_edge = None
        self.future_resp_rm_edge = None
        self.future_resp_battery_edge_rm = None
        self.future_resp_status_platform = None
        self.future_resp_edge_status = None
        self.future_resp_person = None
        self.future_resp_home = None
        self.future_resp_edge_person = None
        self.future_resp_edge_home = None
        self.future_resp_edge_person_rm = None
        self.future_resp_edge_home_rm = None
        self.future_resp_edge_person_seeing = None
        self.future_resp_geozone = None
        self.future_resp_geozone_edge = None

    def read_status_callback(self, msg: PlatformInfo) -> None:
        """Call for status info"""
        print('subscribe to status')
        """Requests"""
        status_name = utils_for_drones.status_from_platform(msg.status.state)
        self.status = utils.node_format('status', node_name=status_name, priority=1)
        req = CreateNode.Request()
        req_edge = CreateEdge.Request()
        print(self.status)
        req.node = self.status
        req_edge.edge = utils.edge_format(
            edge_class='is', source_node=self.dron_node.node_name, target_node=req.node.node_name)
        if self.future_resp_status_platform is None:
            self.future_resp_status_platform = self.cli_create_node.call_async(req)
        if self.future_resp_edge_status is None:
            self.future_resp_edge_status = self.cli_create_edge.call_async(req_edge)

    def read_baterry_state_callback(self, msg: BatteryState) -> None:
        """Call for battery state info"""
        print('subscribe to battery')

        """Requests"""
        # aux_node = utils.node_format_with_prop('Battery', 'Battery', msg.charge, priority=1)
        aux_node = utils.node_format('Battery', 'Battery', priority=1)
        req = CreateNode.Request()
        req.node = aux_node
        req_battery_edge = CreateEdge.Request()
        req_battery_edge_charge = CreateEdge.Request()
        req_battery_edge.edge = utils.edge_format(
            edge_class='high', source_node=self.dron_node.node_name, target_node=req.node.node_name)
        req_battery_edge_charge.edge = utils.edge_format(
            edge_class='low', source_node=self.dron_node.node_name, target_node=req.node.node_name)

        """Futures"""
        if self.future_resp_battery_node is None:
            self.future_resp_battery_node = self.cli_create_node.call_async(req)
        if msg.charge > 50:
            if self.future_resp_battery_edge is None:
                self.future_resp_battery_edge = self.cli_create_edge.call_async(
                    req_battery_edge)
            if self.future_resp_battery_edge_rm is None:
                self.future_resp_battery_edge_rm = self.cli_remove_edge.call_async(
                    req_battery_edge_charge)

        elif self.future_resp_battery_edge is None:
            self.future_resp_battery_edge = self.cli_create_edge.call_async(
                req_battery_edge_charge)
            if self.future_resp_battery_edge_rm is None:
                self.future_resp_battery_edge_rm = self.cli_remove_edge.call_async(
                    req_battery_edge)

    def read_pose_callback(self, msg: PoseStamped) -> None:
        """Call for the pose info topic"""
        print('subscribe to pose')

        self.dron_node = utils.node_format_with_prop(
            'Dron', self.get_namespace(), utils_for_drones.pos_prop_from_pose(msg.pose), priority=1)

        """ Sending request to add node"""
        req = CreateNode.Request()
        req_person = CreateNode.Request()
        req_home = CreateNode.Request()
        req_geozone = CreateNode.Request()
        req_geozone_edge = CreateEdge.Request()
        req.node = self.dron_node
        req_home.node = parameters.node_home
        req_person.node = parameters.node_person
        req_geozone.node = utils_for_drones.node_from_geozone(parameters.geozone)
        req_geozone_edge.edge = utils.edge_format(
            'inside range', self.dron_node.node_name, req_geozone.node.node_name)

        # Creating a future
        if self.future_resp_node is None:
            self.future_resp_node = self.cli_create_node.call_async(req)
        # if self.future_resp_node.done():
        if self.future_resp_person is None:
            self.future_resp_person = self.cli_create_node.call_async(req_person)
        if self.future_resp_home is None:
            self.future_resp_home = self.cli_create_node.call_async(req_home)
        if self.future_resp_geozone is None:
            self.future_resp_geozone = self.cli_create_node.call_async(req_geozone)
        if self.future_resp_geozone_edge is None:
            self.future_resp_geozone_edge = self.cli_create_edge.call_async(
                req_geozone_edge)

    def simulate(self):
        """Requests"""
        req_edge_person_looking = CreateEdge.Request()
        req_edge_home_at = CreateEdge.Request()
        req_edge_person_seeing = CreateEdge.Request()
        req_edge_home_going = CreateEdge.Request()
        req_edge_home_at.edge = utils.edge_format('at', self.dron_node.node_name, 'home')
        req_edge_person_looking.edge = utils.edge_format(
            'looking for', self.dron_node.node_name, 'paco')
        req_edge_person_seeing.edge = utils.edge_format(
            'seeing', self.dron_node.node_name, 'paco')
        req_edge_home_going.edge = utils.edge_format(
            'going to', self.dron_node.node_name, 'home')

        if self.future_resp_edge_home is None:
            self.future_resp_edge_home = self.cli_create_edge.call_async(req_edge_home_at)

        if self.future_resp_edge_person is None:
            self.future_resp_edge_person = self.cli_create_edge.call_async(req_edge_person_looking)

        if self.future_resp_edge_home is not None:
            if self.future_resp_edge_home.done():
                if self.future_resp_edge_home_rm is None:
                    self.future_resp_edge_home_rm = self.cli_remove_edge.call_async(
                        req_edge_home_at)

        if self.future_resp_edge_person is not None:
            if self.future_resp_edge_person.done():
                if self.future_resp_edge_person_rm is None:
                    self.future_resp_edge_person_rm = self.cli_remove_edge.call_async(
                        req_edge_person_looking)

                    self.future_resp_edge_person_seeing = self.cli_create_edge.call_async(
                        req_edge_person_seeing)
        if self.future_resp_edge_person_seeing is not None:
            if self.future_resp_edge_person_seeing.done():
                self.future_resp_edge_home = self.cli_create_edge.call_async(
                    req_edge_home_going)

    def check_futures(self):
        # if self.future_resp_node is not None:
        #     if self.future_resp_node.done():
        #         print('the node dron is create?')
        #         self.future_resp_node = None

        if self.future_resp_status_platform is not None:
            if self.future_resp_status_platform.done():
                print('status update')
                self.future_resp_status_platform = None
        if self.future_resp_edge_status is not None:
            if self.future_resp_edge_status.done():
                print('edge status created')
                self.future_resp_edge_status = None

        if self.future_resp_battery_node is not None:
            if self.future_resp_battery_node.done():
                self.future_resp_battery_node = None
                if self.future_resp_battery_edge is not None:
                    if self.future_resp_battery_edge.done():
                        print('battery edge created')
                        self.future_resp_battery_edge = None

                if self.future_resp_battery_edge_rm is not None:
                    if self.future_resp_battery_edge_rm.done():
                        print('battery rm_edge created')
                        self.future_resp_battery_edge_rm = None
        if self.future_resp_geozone is not None:
            if self.future_resp_geozone.done():
                print('geozone created')
                self.future_resp_geozone = None
                if self.future_resp_geozone_edge is not None:
                    if self.future_resp_geozone_edge.done():
                        self.future_resp_geozone_edge = None
        if self.future_resp_person is not None:
            if self.future_resp_person.done():
                print('the node person is created')
                self.future_resp_person = None
        if self.future_resp_home is not None:
            if self.future_resp_home.done():
                print('the node home is created')
                self.future_resp_home = None


def main():
    rclpy.init()
    service_node = InsertKnowledgeService()
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
