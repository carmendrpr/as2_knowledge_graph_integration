import utils
from geometry_msgs.msg import PoseStamped
from as2_knowledge_graph_msgs.srv import ReadGraph, CreateNode
from knowledge_graph_msgs.msg import Node, Edge, Content, Property
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class IntegrationService(RclNode):

    def __init__(self):
        super().__init__('Implementation_node')
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.read_pose_callback, qos_profile_sensor_data)
        # self.srv = self.create_service(ReadProperty, 'ask_pose', self.ask_pose_callback)

        self.current_pose_ = None
        self.current_node_ = None

    def read_pose_callback(self, msg: PoseStamped):
        """Call for the pose info topic"""
        self.current_pose_ = msg.pose
        # print(self.current_pose_)
        aux_node = Node()
        aux_node.node_name = self.get_namespace()
        prop = utils.pos_prop_from_pose(msg.pose)
        aux_node.properties.append(prop)
        if self.current_pose_.position.x:
            self.current_node_ = aux_node

    def add_node_to_graph(self) -> bool:
        """Add new node to the graph"""
        if self.current_node_ is None:
            return False
        self.cli = self.create_client(CreateNode, 'create_node')
        print("add_node_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = CreateNode.Request()
        self.req.node = self.current_node_

        self.resp = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.resp)
        print("response:", self.resp)
        print(self.req.node.node_name)
        self.current_node_ = None
        return True

    def read_node_from_graph(self, node_name) -> bool:
        print("Read node graph")
        print(node_name)
        if node_name is None:
            return False

        self.cli = self.create_client(ReadGraph, 'read_graph')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ReadGraph.Request()
        self.resp = ReadGraph.Response()
        self.req.node_name = node_name
        self.resp = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.resp)
        for i in (1, 2, 3):
            print("response:", self.resp[i])
        return True


def main():
    rclpy.init()

    service_node = IntegrationService()
    # print(f'node started with ns {service_node.get_namespace()}')
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        service_node.add_node_to_graph()
        service_node.read_node_from_graph('/drone0')

        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
