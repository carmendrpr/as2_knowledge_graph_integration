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
import as2_knowledge_graph_integration.parameters as parameters


def node_format(class_name, node_name, priority) -> Node:
    node = Node()
    node.node_name = node_name
    node.node_class = class_name
    node.properties.append(priority_prop(priority))
    return node


def node_format_with_prop(class_name, node_name, prop, priority) -> Node:
    node = Node()
    node.node_name = node_name
    node.node_class = class_name
    node.properties.append(priority_prop(priority))
    node.properties.append(prop)
    return node


def priority_prop(priority):
    prop = Property()
    prop.key = 'Priority'
    prop.value.type = 1
    prop.value.int_value = priority
    return prop


def edge_format(edge_class, source_node, target_node) -> Edge:
    edge = Edge()
    edge.edge_class = edge_class
    edge.source_node = source_node
    edge.target_node = target_node
    return edge


def look_for_property(list_prop, prop_key) -> Property:
    for aux_prop in list_prop:
        if aux_prop.key == prop_key:
            # print(aux_prop.value.float_vector)
            return aux_prop


def look_for_nodes(list_nodes, node_class):
    nodes = []
    for aux_node in list_nodes:
        if aux_node.node_class == node_class:
            nodes.append(aux_node)
    return nodes


def check_edges(edge, edge_class) -> bool:
    ckeck = False
    if edge.edge_class == edge_class:
        ckeck = True
    return ckeck
