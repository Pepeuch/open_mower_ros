cmake_minimum_required(VERSION 3.0.2)
project(mavros_package)

## Rechercher les paquets ROS requis
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_msgs
)

## Ajouter les informations de configuration pour catkin
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mavros mavros_msgs
)

## Ajouter les répertoires d'inclusion
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Déclaration des nœuds (exemple)
# add_executable(my_node src/my_node.cpp)
# target_link_libraries(my_node ${catkin_LIBRARIES})
