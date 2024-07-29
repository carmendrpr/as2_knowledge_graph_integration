import as2_knowledge_graph_integration.utils as utils
import as2_knowledge_graph_integration.utils_for_drones as utils_for_drones
import as2_knowledge_graph_integration.parameters as parameters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from as2_msgs.msg import PlatformInfo
from as2_knowledge_graph_msgs.srv import CreateNode, CreateEdge
import rclpy
from rclpy.node import Node as RclNode
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class ExtractKnowledgeService(RclNode):
    def __init__(self):
        super().__init__('insert_knowledge_node')


def main():
    rclpy.init()
    service_node = ExtractKnowledgeService()
    import time

    while rclpy.ok():

        rclpy.spin_once(service_node)
        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
