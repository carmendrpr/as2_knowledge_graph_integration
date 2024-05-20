from geometry_msgs.msg import PoseStamped
from as2_knowledge_graph_msgs.srv import ReadGraph, CreateNode
from knowledge_graph_msgs.msg import Node, Edge, Content, Property
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class IntegrationService(RclNode):

    def __init__(self):
        # # Global items
        # current_pose_ = PoseStamped()
        # current_node_ = Node()

        super().__init__('Implementation_node')
        self.subscription = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.read_pose_callback, qos_profile_sensor_data)
        # self.cli = self.create_client(CreateNode, 'create_node')
        self.cli = self.create_client(ReadGraph, 'service_read_node_graph_')
        # self.srv = self.create_service(ReadProperty, 'ask_pose', self.ask_pose_callback)

        self.current_pose_ = None
        self.current_node_ = None

    def read_pose_callback(self, msg: PoseStamped):
        """Call for the pose info topic"""
        self.current_pose_ = msg.pose
        print(self.current_pose_)

        aux_node = Node()
        aux_node.node_name = self.get_namespace()
        # aux_node.properties = xpose
        if self.current_pose_.position.x:
            self.current_node_ = aux_node

    def add_node_(self) -> bool:
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
        self.current_node_ = None
        return self.resp

    # if (self.current_node_):
    #     add_node_(current_node_)

    #     if read_pose_callback:
    #         add_node_(self, current_pose_)

    #     def check_pose(self) -> bool:
    #         """Check if the current pose is the desired pose, pose=1m"""
    #         if (self.current_pose_.pose.position.z) == 1:
    #             return True
    #         return False

    #     def ask_pose_callback(self, request, response):
    #         for i in
    #         response
    #     self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    #     return response


def main():
    rclpy.init()

    service_node = IntegrationService()
    # print(f'node started with ns {service_node.get_namespace()}')
    import time

    while rclpy.ok():
        rclpy.spin_once(service_node)
        out = service_node.add_node_()
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
