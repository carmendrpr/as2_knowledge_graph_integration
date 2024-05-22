cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME as2_knowledge_graph_integration)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE Debug)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclcpp
  backward_ros
  knowledge_graph
  knowledge_graph_msgs
  rosidl_default_generators
  ament_cmake_gtest
  as2_core
  as2_knowledge_graph_msgs
  as2_knowledge_graph
  as2_knowledge_graph_integration_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()



# Install python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(DIRECTORY
  ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(tests)
endif()

ament_package(

)