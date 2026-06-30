cmake_minimum_required(VERSION 3.8)
project(sentry_behavior)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sentry_behavior.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
