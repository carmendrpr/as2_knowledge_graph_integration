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

import math
from knowledge_graph_msgs.msg import Node, Edge, Content, Property

"""Node person"""
node_person = Node()
node_person.node_class = 'person'
node_person.node_name = 'paco'
prop_person = Property()
prop_person.key = 'Position'
prop_person.value.type = 7
prop_person.value.float_vector.extend([2.5, 3, 1.2])
node_person.properties.append(prop_person)

"""Node Home"""
node_home = Node()
node_home.node_class = 'location'
node_home.node_name = 'home'
prop_home = Property()
prop_home.key = 'Position'
prop_home.value.type = 7
prop_home.value.float_vector.extend([0, 0, 0])
node_home.properties.append(prop_home)


def pos_prop_from_pose(pose):
    prop = Property()
    prop.key = 'Position'
    # prop.value.type = Property.VFLOAT
    prop.value.type = 7
    prop.value.float_vector.append(pose.position.x)
    prop.value.float_vector.append(pose.position.y)
    prop.value.float_vector.append(pose.position.z)
    return prop


def battery_prop_from_msg(battery):
    prop = Property()
    prop.key = 'Battery'
    prop.value.type = 2
    prop.value.float_value = battery
    return prop


def status_from_platform(platform_info) -> str:
    node_name = str()
    if platform_info == 1:
        node_name = 'Landed'
    elif platform_info == 2:
        node_name = 'Taking off'
    elif platform_info == 3:
        node_name = 'Flying'
    elif platform_info == 4:
        node_name = 'Landing'
    return node_name


def priority_prop(priority):
    prop = Property()
    prop.key = 'Priority'
    prop.value.type = 1
    prop.value.int_value = priority
    return prop


def node_from_msg(class_name, node_name, msg, priority) -> Node:
    node = Node()
    node.node_name = node_name
    node.node_class = class_name
    node.properties.append(priority_prop(priority))
    node.properties.append(pos_prop_from_pose(msg))
    return node


def node_format(class_name, node_name, priority) -> Node:
    node = Node()
    node.node_name = node_name
    node.node_class = class_name
    node.properties.append(priority_prop(priority))
    return node


def node_format_with_prop(class_name, node_name, msg, priority) -> Node:
    node = Node()
    node.node_name = node_name
    node.node_class = class_name
    node.properties.append(priority_prop(priority))
    # node.properties.append(battery_prop_from_msg(msg))
    return node


def edge_format(edge_class, source_node, target_node) -> Edge:
    edge = Edge()
    edge.edge_class = edge_class
    edge.source_node = source_node
    edge.target_node = target_node
    return edge


def calculate_distance(node_prop_1, node_prop_2) -> float:
    aux_prop_1 = node_prop_1
    aux_prop_2 = node_prop_2
    distance = 0
    aux = 0
    for i in range(2):
        aux = ((aux_prop_1.value.float_vector[i]) -
               (aux_prop_2.value.float_vector[i]))**2
        distance = distance + aux
    return math.sqrt(distance)


def look_for_property(list_prop, prop_key) -> Property:
    for aux_prop in list_prop:
        if aux_prop.key == prop_key:
            print(aux_prop.value.float_vector)
            return aux_prop


def look_for_nodes(list_nodes, node_class):
    nodes = []
    for aux_node in list_nodes:
        if aux_node.node_class == node_class:
            nodes.append(aux_node)
    return nodes
