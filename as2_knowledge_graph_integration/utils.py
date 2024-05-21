from knowledge_graph_msgs.msg import Property, Node


def pos_prop_from_pose(pose):
    prop = Property()
    prop.key = 'Position'
    # prop.value.type = Property.VFLOAT
    prop.value.type = 7
    prop.value.float_vector.append(pose.position.x)
    prop.value.float_vector.append(pose.position.y)
    prop.value.float_vector.append(pose.position.z)
    return prop


def node_from_msg(class_name, name, msg) -> Node:
    node = Node()
    node.node_name = name
    node.node_class = class_name
    node.properties.append(pos_prop_from_pose(msg))
    return node
