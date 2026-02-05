cmake_minimum_required(VERSION 3.8)
project(wheeltec_description)

find_package(ament_cmake REQUIRED)

# Install urdf files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install meshes (if any)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  OPTIONAL
)

ament_package()
