cmake_minimum_required(VERSION 3.0.2)
project(wrist_control)

# Si no, da un problema al incluir dynamixel_sdk.h
include_directories(/home/ordavidro/wrist_ws/src/DynamixelSDK/ros/dynamixel_sdk/include/dynamixel_sdk)
include_directories(/home/ordavidro/wrist_ws/src/dynamixel_ros_library/include)

# Compilar como C++11, soportado en ROS Kinetic y más recientes
add_compile_options(-std=c++11)

# Encontrar macros y librerías de catkin
# Si se usa una lista de COMPONENTES como find_package(catkin REQUIRED COMPONENTS xyz)
# también encontrar otros paquetes de catkin
find_package(catkin REQUIRED COMPONENTS
  dynamixel_ros_library
  dynamixel_sdk
  roscpp
  std_msgs
  message_generation
)


# Declaración del paquete catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_ros_library
  CATKIN_DEPENDS roscpp dynamixel_sdk dynamixel_ros_library std_msgs
)

# Especificar ubicaciones adicionales de archivos de cabecera
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#Añadir ejecutables
add_executable(ControlTopic src/ControlTopic.cpp)
target_link_libraries(ControlTopic ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES} ${dynamixel_ros_library_LIBRARIES})
add_dependencies(ControlTopic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#Añadir ejecutables
add_executable(ControlImu src/ControlImu.cpp)
target_link_libraries(ControlImu ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES} ${dynamixel_ros_library_LIBRARIES})
add_dependencies(ControlImu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Marcar ejecutables para instalación
install(TARGETS ControlTopic ControlImu
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Marcar archivos de cabecera cpp para instalación
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)