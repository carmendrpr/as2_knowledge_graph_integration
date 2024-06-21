import rclpy
import as2_knowledge_graph_integration.utils as utils
from rclpy.node import Node as RclNode
from as2_knowledge_graph_msgs.srv import ReadGraph, ReadEdgeGraph
from knowledge_graph_msgs.msg import Node


class ReadMyGraph(RclNode):
    def __init__(self):
        super().__init__('read_graph')

        """
        Timer
        """

        """
        Clients
        """
        self.client_nodes = self.create_client(ReadGraph, '/read_node_graph')
        self.client_edges = self.create_client(ReadEdgeGraph, '/read_edge_class_graph')

        while not self.client_nodes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_graph is not available, waiting again...')
        while not self.client_edges.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_graph is not available, waiting again...')

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
