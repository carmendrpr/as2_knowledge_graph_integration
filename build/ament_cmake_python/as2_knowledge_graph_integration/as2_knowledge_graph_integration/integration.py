import utils
import time
from geometry_msgs.msg import PoseStamped
from as2_knowledge_graph_msgs.srv import ReadGraph, CreateNode
from knowledge_graph_msgs.msg import Node, Edge, Content, Property
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class IntegrationService(RclNode):

    def __init__(self):
        super().__init__('Implementation_node')

        """Timers"""
        self.timer_1 = self.create_timer(3.0, self.create_node_callback)

        """Subscription"""
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.read_pose_callback, qos_profile_sensor_data)

        """Clients"""
        self.cli_create_node = self.create_client(CreateNode, 'create_node')

        # Checking if the services are available
        while not self.cli_create_node.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.current_pose_ = None
        self.current_node_ = None

    def read_pose_callback(self, msg: PoseStamped) -> None:
        """Call for the pose info topic"""
        print('suscribe to pose')
        aux_node = utils.node_from_msg('Dron', self.get_namespace(), msg.pose)
        # list_req = [aux_node, 'volando']
        self.current_node_ = aux_node
        # self.future_resp = CreateNode()
        # if not self.future_resp is None:
        # Sending request to add node
        req = CreateNode.Request()
        req.node = self.current_node_
        self.future_resp = self.cli_create_node.call_async(req)

    def create_node_callback(self):
        self.get_logger().info('Timer 1 executing')
        # checking if resp is done
        if self.future_resp.done():
            print('the reponse is', self.future_resp.result())

    # def read_node_from_graph(self, node_name) -> bool:
    #     print("Read node graph")
    #     print(node_name)
    #     if node_name is None:
    #         return False

    #     self.cli_read_graph = self.create_client(ReadGraph, 'read_graph')
    #     while not self.cli_read_graph.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     self.req = ReadGraph.Request()
    #     self.resp = ReadGraph.Response()
    #     self.req.node_name = node_name
    #     self.resp = self.cli_read_graph.call_async(self.req)
    #     # rclpy.spin_until_future_complete(self, self.resp)

    #     return True


def main():
    rclpy.init()

    service_node = IntegrationService()
    # print(f'node started with ns {service_node.get_namespace()}')
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
