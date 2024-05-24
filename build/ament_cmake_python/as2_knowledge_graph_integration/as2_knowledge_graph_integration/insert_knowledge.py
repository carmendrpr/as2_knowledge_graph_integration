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


import utils
import time
from geometry_msgs.msg import PoseStamped
from as2_knowledge_graph_msgs.srv import CreateNode, CreateEdge
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class InsertKnowledgeService(RclNode):

    def __init__(self):
        super().__init__('insert_knowledge_node')

        """Timers"""
        self.timer_1 = self.create_timer(3.0, self.create_node_callback)
        self.timer_2 = self.create_timer(3.0, self.create_node_flying_callback)
        self.timer_3 = self.create_timer(6.0, self.create_edge_callback)
        self.timer_4 = self.create_timer(3.0, self.remove_edge_calback)

        """Subscription"""
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.read_pose_callback, qos_profile_sensor_data)

        """Clients"""
        self.cli_create_node = self.create_client(CreateNode, '/create_node')
        self.cli_create_node_status = self.create_client(CreateNode, '/create_node')
        self.cli_create_edge = self.create_client(CreateEdge, '/create_edge')
        self.cli_remove_edge = self.create_client(CreateEdge, '/remove_edge')

        # Checking if the services are available
        while not self.cli_create_node.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_node is not available, waiting again...')
        while not self.cli_create_node_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_node is not available, waiting again...')
        while not self.cli_create_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_edge is not available, waiting again...')
        while not self.cli_remove_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service remove_edge is not available, waiting again...')

        # ¿podria crear una lista para integrar todas estas variables?
        self.current_pose_ = None
        self.current_node_ = None
        self.future_resp_node = None
        self.future_resp_status = None
        self.future_resp_edge = None
        self.future_resp_rm_edge = None

    def read_pose_callback(self, msg: PoseStamped) -> None:
        """Call for the pose info topic"""
        print('suscribe to pose')
        aux_node = utils.node_from_msg('Dron', self.get_namespace(), msg.pose)

        # Sending request to add node
        req = CreateNode.Request()
        req_status = CreateNode.Request()
        req_edge = CreateEdge.Request()

        req.node = aux_node
        req_status.node = utils.node_format(class_name="status", node_name="flying")
        req_edge.edge = utils.edge_format(
            edge_class='is', source_node=aux_node.node_name, target_node=req_status.node.node_name)

        # Creating a future
        if self.future_resp_node is None:
            self.future_resp_node = self.cli_create_node.call_async(req)

        if msg.pose.position.z > 0.5:
            if self.future_resp_status is None:
                self.future_resp_status = self.cli_create_node_status.call_async(req_status)
            if self.future_resp_edge is None:
                self.future_resp_edge = self.cli_create_edge.call_async(req_edge)

        if msg.pose.position.z < 0.5:
            if self.future_resp_rm_edge is None:
                self.future_resp_rm_edge = self.cli_remove_edge.call_async(req_edge)

    def create_node_callback(self):
        self.get_logger().info('Waiting for create_node ')
        # checking if resp is done
        if self.future_resp_node is not None:
            if self.future_resp_node.done():
                print('the node dron is created', self.future_resp_node.result())
                self.future_resp_node = None

    def create_node_flying_callback(self):
        self.get_logger().info('Waiting for create_node_flying')
        if self.future_resp_status is not None:
            if self.future_resp_status.done():
                print('the node volando is created', self.future_resp_status.result())
                self.future_resp_status = None

    def create_edge_callback(self):
        self.get_logger().info('Waiting for create_edge')
        if self.future_resp_edge is not None:
            if self.future_resp_edge.done():
                print('the edge is created', self.future_resp_edge.result())
                self.future_resp_edge = None

    def remove_edge_calback(self):
        self.get_logger().info('Waiting for remove_edge')
        if self.future_resp_rm_edge is not None:
            if self.future_resp_rm_edge.done():
                print('the edge is created', self.future_resp_rm_edge.result())
                self.future_resp_rm_edge = None


def main():
    rclpy.init()

    service_node = InsertKnowledgeService()
    # print(f'node started with ns {service_node.get_namespace()}')
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
