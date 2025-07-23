cmake_minimum_required(VERSION 3.5)
project(bopa_logger)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/logger_collector")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            src/logger_collector   # target folder
            logger_collector       # link name (no slash!)
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/nodes")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            src/nodes
            nodes
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 1) Install Python package from src/logger_collector
ament_python_install_package(
  logger_collector
)

# 2) Copy run.py stub into libexec/bopa_logger/
install(PROGRAMS
  nodes/run.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run.py
)

# 3) Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
