import rclpy
from rclpy.node import Node as RclNode
from as2_knowledge_graph_msgs.srv import ReadGraph, ReadEdgeGraph


class ReadMyGraph(RclNode):
    def __init__(self):
        super().__init__('read_graph')

        """
        Timer
        """
        self.timer = self.create_timer(0.5, self.read_graph)
        self.timer = self.create_timer(2.0, self.future_callbacks)

        """
        Clients
        """
        self.client_nodes = self.create_client(ReadGraph, '/read_node_graph')
        self.client_edges = self.create_client(ReadEdgeGraph, '/read_edge_class_graph')

        while not self.client_nodes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_graph is not available, waiting again...')
        while not self.client_edges.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service read_graph is not available, waiting again...')

        self.resp = None
        self.responses = {}

        self.request_edge = ReadEdgeGraph.Request()

    def read_graph(self):
        # self.request_edge.source_node = '/drone0'
        # self.request_edge.target_node = 'Battery'
        # req_edge = ReadEdgeGraph.Request()
        # req_edge.source_node = '/drone0'
        # req_edge.target_node = 'Battery'
        # req_edge.edge_class = ''
        # if self.responses.get('bat_levl') is None:
        #     print(self.responses.get('bat_levl'))
        #     # self.responses.update('bat_level', self.client_edges.call_async(
        #     #     self.request_edge.source_node, self.request_edge.target_node))
        #     self.responses.update({'bat_level': self.client_edges.call_async(
        #         req_edge)})
        #     print(self.responses.get('bat_levl'))
        req_nodes = ReadGraph.Request()
        req_nodes.node_class = 'Dron'

        if self.responses.get('nodes') is None:
            self.responses.update({'nodes': self.client_nodes.call_async(req_nodes)})
            # print(self.responses.get('nodes'))

        """esto funciona"""
        # if self.resp is None:
        #     self.resp = self.client_nodes.call_async(req_nodes)

    def future_callbacks(self):
        # if self.responses.get('bat_levl') is not None:
        #     if self.responses.get('bat_levl').done():
        #         print(self.responses.get('bat_levl'))
        # self.responses.update({'bat_level': None})

        if self.responses.get('nodes') is not None:
            if self.responses['nodes'].done():
                print(self.responses['nodes'].result())
        """esto funciona"""
        # if self.resp is not None:
        #     if self.resp.done():
        #         responses = ReadGraph.Response()
        #         responses = self.resp.result()
        #         print(responses)
        #         self.resp = None


def main():
    rclpy.init()
    service_node = ReadMyGraph()
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)

        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
