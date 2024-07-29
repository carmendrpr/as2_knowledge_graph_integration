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

from knowledge_graph_msgs.msg import Node, Property

"""Node person"""
node_person = Node()
node_person.node_class = 'Person'
node_person.node_name = 'paco'
prop_person = Property()
prop_person.key = 'Position'
prop_person.value.type = 7
prop_person.value.float_vector.extend([2, 1.5, 1.2])
node_person.properties.append(prop_person)

"""Node Home"""
node_home = Node()
node_home.node_class = 'location'
node_home.node_name = 'home'
prop_home = Property()
prop_home.key = 'Position'
prop_home.value.type = 7
prop_home.value.float_vector.extend([1, 1, 0])
node_home.properties.append(prop_home)

"""GEOZONE"""
geozone = {
    'Grid reference': {
        (0, 0): 7, (0, 1): 0, (0, 2): 0,
        (1, 0): 0, (1, 1): 7, (1, 2): 0,
        (2, 0): 0, (2, 1): 0, (2, 2): 7
    },
    'Warning zone': {
        (0, 0): 9, (0, 1): 0, (0, 2): 0,
        (1, 0): 0, (1, 1): 9, (1, 2): 0,
        (2, 0): 0, (2, 1): 0, (2, 2): 9
    }
}
