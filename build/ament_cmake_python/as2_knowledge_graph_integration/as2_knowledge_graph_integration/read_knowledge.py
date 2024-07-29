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
import rclpy
from rclpy.node import Node as RclNode
from as2_knowledge_graph_msgs.srv import ReadGraph, CreateEdge, ReadEdgeGraph
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class ReadKnowledgeService(RclNode):

    def __init__(self):
        super().__init__('read_knowledge_node')
        """Timers"""
        self.timer = self.create_timer(0.5, self.general_calback)
        self.timer_check_distance = self.create_timer(1.0, self.check_distance_callback)
        self.timer_check_edge = self.create_timer(1.0, self.check_edge)
        """Clients"""
        self.cli_read_nodes_of_type_dron = self.create_client(ReadGraph, '/read_node_graph')
        self.cli_create_edge_near = self.create_client(CreateEdge, '/create_edge')
        self.cli_exist_edge = self.create_client(ReadEdgeGraph, '/read_edge_source_target_graph')
        self.cli_remove_edge = self.create_client(CreateEdge, '/remove_edge')

    # Checking if the services are available
        while not self.cli_read_nodes_of_type_dron.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_nodes is not available, waiting again...')

        while not self.cli_create_edge_near.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service create_edge is not available, waiting again...')

        while not self.cli_exist_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_edge_source_target_graph is not available, waiting again...')

        while not self.cli_remove_edge.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service remove_edge is not available, waiting again...')

    # Create future response
        self.future_resp_nodes_info = None
        self.future_resp_create_edge = None
        self.future_resp_exist_edge = None
        self.future_resp_remove_edge = None

    def general_calback(self) -> None:
        """Main callback"""
        aux = ReadGraph.Request()
        aux.node_class = 'Dron'
        if self.future_resp_nodes_info is None:
            self.future_resp_nodes_info = self.cli_read_nodes_of_type_dron.call_async(aux)

    def check_distance_callback(self) -> None:
        self.get_logger().info('Waiting to read the graph')
        if self.future_resp_nodes_info is not None:
            if self.future_resp_nodes_info.done():
                response = ReadGraph.Response()
                response = self.future_resp_nodes_info.result()
                nodes = response.nodes
                print(f'lenght : {len(nodes)}')
                for index, node in enumerate(nodes):
                    for index_aux in range(index + 1, len(nodes)):
                        aux_node = nodes[index_aux]
                        aux_prop_1 = utils.look_for_property(node.properties, 'Position')
                        aux_prop_2 = utils.look_for_property(aux_node.properties, 'Position')
                        self.distance = utils_for_drones.calculate_distance(
                            aux_prop_1, aux_prop_2)
                        print('the distance is', self.distance)
                        req_aux_edge = CreateEdge.Request()
                        req_exist_edge = ReadEdgeGraph.Request()
                        req_aux_edge.edge = utils.edge_format(
                            edge_class='near', source_node=node.node_name, target_node=aux_node.node_name)
                        req_exist_edge.edge_class = "near"
                        if self.distance < 0.7:
                            if self.future_resp_create_edge is None:
                                self.future_resp_create_edge = self.cli_create_edge_near.call_async(
                                    req_aux_edge)
                        elif self.future_resp_remove_edge is None:
                            self.future_resp_remove_edge = self.cli_remove_edge.call_async(
                                req_aux_edge)

                self.future_resp_nodes_info = None

    def check_edge(self):
        if self.future_resp_create_edge is not None:
            if self.future_resp_create_edge.done():
                self.future_resp_create_edge = None
                print("Succssefully create the edge")
        if self.future_resp_remove_edge is not None:
            if self.future_resp_remove_edge.done():
                self.future_resp_remove_edge = None


def main():
    rclpy.init()

    service_node = ReadKnowledgeService()
    # print(f'node started with ns {service_node.get_namespace()}')
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
