# as2_knowledge_graph_integration
1) First launch
     aerostack2/project_gazebo
2) launch 3 drones
     ./launch_as2.bash -m -t
3)In other terminal launch rqt
4)In other terminal launch the graph
    ros2 run as2_knowledge_graph as2_knowledge_graph_node_server_main
5)~/ros2_ws/src/as2_knowledge_graph_integration
  5.1)launch one service for each drone with their namespace
       python3 as2_knowledge_graph_integration/integration.py --ros-args -r __ns:='/<namespace>'

  
