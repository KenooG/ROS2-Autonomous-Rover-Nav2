cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY
  models
  worlds
  config   
  launch   
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/ocr_detector.py
  scripts/random_swap.py
  DESTINATION lib/${PROJECT_NAME}
)


install(FILES
  scripts/frozen_east_text_detection.pb
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()