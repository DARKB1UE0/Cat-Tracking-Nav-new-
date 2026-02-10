cmake_minimum_required(VERSION 3.8)
project(wheeltec_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install maps directory
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  OPTIONAL
)

# Install scripts
install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

