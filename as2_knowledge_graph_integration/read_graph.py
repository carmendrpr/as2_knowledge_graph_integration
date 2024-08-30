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


import rclpy
import as2_knowledge_graph_integration.utils as utils
from rclpy.node import Node as RclNode
from as2_knowledge_graph_msgs.srv import ReadGraph, ReadEdgeGraph
from knowledge_graph_msgs.msg import Node


class ReadMyGraph(RclNode):
    def __init__(self):
        super().__init__('read_graph')

        """
        Clients
        """
        self.client_nodes = self.create_client(ReadGraph, '/read_node_graph')
        self.client_edges = self.create_client(ReadEdgeGraph, '/read_edge_class_graph')

        while not self.client_nodes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_graph is not available, waiting again...')
        while not self.client_edges.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_edge_graph is not available, waiting again...')

    def query_graph_node(self, req: ReadGraph.Request) -> ReadGraph.Response:
        future = self.client_nodes.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def query_graph_edge(self, req: ReadEdgeGraph.Request) -> ReadEdgeGraph.Response:
        future = self.client_edges.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def query_node_name(self, node_class) -> Node.node_name:
        req = ReadGraph.Request()
        req.node_class = node_class
        nodes = ReadGraph.Response()
        #    return a list of nodes
        nodes = self.query_graph_node(req)
        if not nodes.nodes:
            return None
        return nodes.nodes[0].node_name

    def check_status_of_edge(self, source_node_class, target_node_class, edge_name) -> bool:
        req = ReadEdgeGraph.Request()
        if source_node_class is None or target_node_class is None or edge_name is None:
            return False
        if self.query_node_name(source_node_class) is None or self.query_node_name(target_node_class) is None:
            return False

        req.source_node = self.query_node_name(source_node_class)
        req.target_node = self.query_node_name(target_node_class)
        resp = self.query_graph_edge(req)
        if not resp.edge:
            print('empty')
            return False
        status_aux = utils.check_edges(resp.edge[0], edge_name)

        return status_aux


def main():
    rclpy.init()
    service_node = ReadMyGraph()

    out = service_node.check_status_of_edge('Dron', 'Battery', 'high')
    print(f'{out=}')
    out2 = service_node.query_node_name('Dron')
    print(f'{out2=}')
    out3 = service_node.check_status_of_edge('Dron', 'Person', 'seeing')
    print(f'{out3=}')


if __name__ == '__main__':
    main()
