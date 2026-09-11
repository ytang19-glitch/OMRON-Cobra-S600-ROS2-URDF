cmake_minimum_required(VERSION 3.8)
project(omron_cobra_s600_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/control_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
