cmake_minimum_required(VERSION 3.0.2)
project(alarm)

## 编译时需要依赖的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
)

## 生成动态重配置的配置文件
generate_dynamic_reconfigure_options(
  cfg/ObstacleDetection.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 指定C++编译选项，可选
## add_executable(${PROJECT_NAME}_node src/your_node.cpp)
## target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
## add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定Python节点的安装选项
catkin_install_python(PROGRAMS scripts/alarm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

