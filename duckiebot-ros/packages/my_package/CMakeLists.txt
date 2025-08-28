cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

# Install Python scripts
catkin_install_python(
  PROGRAMS 
    src/camera_reader_node.py
    src/my_publisher_node.py
    src/my_subscriber_node.py
    src/square_controller_node.py
    src/twist_control_node.py
    src/wheel_control_node.py
    src/wheel_encoder_reader_node.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
