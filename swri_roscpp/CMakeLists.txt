cmake_minimum_required(VERSION 3.5)

project(swri_roscpp)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(INCLUDE_DIRS include ${${BUILD_DEPS}_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${${BUILD_DEPS}_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${${BUILD_DEPS}_LIBRARIES} ${ament_cmake_LIBRARIES})

set(BUILD_DEPS diagnostic_updater nav_msgs rclcpp std_msgs std_srvs)

set(RUNTIME_DEPS diagnostic_updater nav_msgs rclcpp std_msgs std_srvs)

### Catkin ###

### Build Test Node ###
#add_executable(subscriber_test src/nodes/subscriber_test.cpp)
#target_link_libraries(subscriber_test ${LIBS})

#add_executable(latched_subscriber_test src/nodes/latched_subscriber_test.cpp)
#target_link_libraries(latched_subscriber_test ${LIBS})

#add_executable(storing_subscriber_test src/nodes/storing_subscriber_test.cpp)
#target_link_libraries(storing_subscriber_test ${LIBS})

#add_executable(service_server_test src/nodes/service_server_test.cpp)
#target_link_libraries(service_server_test ${LIBS})

#add_executable(timer_test src/nodes/timer_test.cpp)
#target_link_libraries(timer_test ${LIBS})

#add_executable(param_example src/nodes/param_example.cpp)
#target_link_libraries(param_example ${LIBS})

add_library(${PROJECT_NAME}
  src/logging.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS})

if(BUILD_TESTING)
  #find_package(rostest REQUIRED)
  #add_rostest_gtest(test_params test/params.test test/params.cpp)
  #target_link_libraries(test_params ${LIBS})
endif()

### Install Test Node and Headers ###
install(TARGETS 
  ${PROJECT_NAME}
  #subscriber_test latched_subscriber_test param_example
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${BUILD_DEPS})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

ament_package()
