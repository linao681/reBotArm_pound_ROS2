cmake_minimum_required(VERSION 3.22)
project(rebotarm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
