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
import as2_knowledge_graph_integration.utils as utils
from knowledge_graph_msgs.msg import Node, Property
from as2_knowledge_graph_msgs.srv import CreateEdge
import as2_knowledge_graph_integration.parameters as parameters


def node_from_geozone(geozone):
    geozone = parameters.geozone
    node = Node()
    node.node_class = 'Position'
    node.node_name = 'geozone'
    prop = Property()
    keys = list(geozone.keys())
    prop.key = keys[0]
    prop.value.type = 7
    vec1 = list(geozone['Grid reference'].values())
    prop.value.float_vector.extend(vec1)
    node.properties.append(prop)
    prop2 = Property()
    prop2.key = keys[1]
    vec2 = list(geozone['Warning zone'].values())
    prop2.value.float_vector.extend(vec2)
    node.properties.append(prop2)
    return node


def pos_prop_from_pose(pose):
    prop = Property()
    prop.key = 'Position'
    prop.value.type = 7
    prop.value.float_vector.append(pose.position.x)
    prop.value.float_vector.append(pose.position.y)
    prop.value.float_vector.append(pose.position.z)
    return prop


def charge_prop_from_battery(battery):
    prop = Property()
    prop.key = 'Charge'
    prop.value.type = 2
    prop.value.float_value = battery.charge
    return prop


def status_from_platform(platform_info) -> str:
    node_name = str()
    if platform_info == 1:
        node_name = 'Landed'
    elif platform_info == 2:
        node_name = 'Flying'
    elif platform_info == 3:
        node_name = 'Flying'
    elif platform_info == 4:
        node_name = 'Flying'
    elif platform_info == 0:
        node_name = 'Landed'
    return node_name


def batery_edge_req(battery_charge, source_node, target_node):
    batery_edge = CreateEdge.Request()
    if battery_charge > 50:
        batery_edge.edge = utils.edge_format(
            edge_class='high', source_node=source_node, target_node=target_node)
    elif battery_charge < 50:
        batery_edge.edge = utils.edge_format(
            edge_class='low', source_node=source_node, target_node=target_node)
    return batery_edge.edge


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


def person_edge_req(source_node, target_node, ns):
    person_edge = CreateEdge.Request()
    person_edge_rm = CreateEdge.Request()
    pose_prop = utils.look_for_property(source_node.properties, 'Position')
    person_prop = utils.look_for_property(target_node.properties, 'Position')
    distance = calculate_distance(pose_prop, person_prop)
    if distance < 0.8:
        person_edge.edge = utils.edge_format(
            edge_class='seeing', source_node=ns, target_node=target_node.node_name)
        person_edge_rm.edge = utils.edge_format(
            edge_class='looking for', source_node=ns, target_node=target_node.node_name)
        return person_edge.edge, person_edge_rm.edge
    return None


def person_edge_saw_req(source_node, target_node):
    person_edge = CreateEdge.Request()
    person_edge_rm = CreateEdge.Request()
    person_edge.edge = utils.edge_format(
        edge_class='located', source_node=source_node.node_name, target_node=target_node.node_name)
    person_edge_rm.edge = utils.edge_format(
        edge_class='seeing', source_node=source_node.node_name, target_node=target_node.node_name)
    return person_edge.edge, person_edge_rm.edge


def home_edge_req(source_node, target_node, flag):
    home_edge = CreateEdge.Request()
    home_edge_rm = CreateEdge.Request()

    pose_prop = utils.look_for_property(source_node.properties, 'Position')
    home_prop = utils.look_for_property(target_node.properties, 'Position')
    distance = calculate_distance(pose_prop, home_prop)
    if flag:
        if distance < 0.02:
            home_edge.edge = utils.edge_format(
                edge_class='is at', source_node=source_node.node_name, target_node=target_node.node_name)
            home_edge_rm.edge = utils.edge_format(
                edge_class='going', source_node=source_node.node_name, target_node=target_node.node_name)
        else:
            home_edge.edge = utils.edge_format(
                edge_class='going', source_node=source_node.node_name, target_node=target_node.node_name)
            home_edge_rm.edge = utils.edge_format(
                edge_class='outside', source_node=source_node.node_name, target_node=target_node.node_name)

    else:
        if distance < 0.02:
            home_edge.edge = utils.edge_format(
                edge_class='is at', source_node=source_node.node_name, target_node=target_node.node_name)
            home_edge_rm.edge = utils.edge_format(
                edge_class='outside', source_node=source_node.node_name, target_node=target_node.node_name)
        else:
            home_edge.edge = utils.edge_format(
                edge_class='outside', source_node=source_node.node_name, target_node=target_node.node_name)
            home_edge_rm.edge = utils.edge_format(
                edge_class='is at', source_node=source_node.node_name, target_node=target_node.node_name)

    return home_edge.edge, home_edge_rm.edge


def geozone_edge_req(source_node, target_node, ns):
    geozone_edge = CreateEdge.Request()
    pose_prop = utils.look_for_property(source_node.properties, 'Position')
    geozone_prop_grid = utils.look_for_property(target_node.properties, 'Grid reference')
    geozone_prop_warning = utils.look_for_property(target_node.properties, 'Warning zone')
    print(geozone_prop_grid)
    print(geozone_prop_warning)
    # distance = calculate_distance(pose_prop, geozone_prop)
    # if distance == 0:
    #     geozone_edge.edge = utils.edge_format(
    #         edge_class='is at', source_node=ns, target_node=target_node.node_name)
    # else:
    #     geozone_edge.edge = utils.edge_format(
    #         edge_class='outside', source_node=ns, target_node.node_name)
    # return geozone_edge.edge


def check_distances(node1, node2) -> bool:
    pose1 = utils.look_for_property(node1.properties, 'Position')
    pose2 = utils.look_for_property(node2.properties, 'Position')
    distances = calculate_distance(pose1, pose2)

    if distances < 0.3:
        return True

    else:
        return False
