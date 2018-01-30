cmake_minimum_required(VERSION 3.5)

project(swri_roscpp)

# Add support for C++11
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 11)
#endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosgraph_msgs REQUIRED)

set(INCLUDE_DIRS include ${${BUILD_DEPS}_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${${BUILD_DEPS}_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${${BUILD_DEPS}_LIBRARIES} ${ament_cmake_LIBRARIES})

set(BUILD_DEPS nav_msgs rclcpp std_msgs std_srvs class_loader)

set(RUNTIME_DEPS nav_msgs rclcpp std_msgs std_srvs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Interrogate.srv"
  "srv/LoadNode.srv"
)

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

add_executable(nodelet
  src/nodelet.cpp
)
ament_target_dependencies(nodelet
  class_loader
  rcutils
  rclcpp
)
target_link_libraries(nodelet 
  ${LIBS} 
  ${PROJECT_NAME}_core 
  ament_index_cpp::ament_index_cpp
)
rosidl_target_interfaces(nodelet
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)


add_executable(nodelet_manager src/nodelet_manager.cpp)
ament_target_dependencies(nodelet_manager
  class_loader
  rcutils
  rclcpp
  rosidl_typesupport_cpp
)
rosidl_target_interfaces(nodelet_manager
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)
target_link_libraries(nodelet_manager 
  ${LIBS} 
  ${PROJECT_NAME}_core
  ${rosidl_typesupport_cpp_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

add_executable(node_info src/node_info.cpp)
ament_target_dependencies(node_info
  rcutils
  rclcpp
  rosidl_typesupport_cpp
)
rosidl_target_interfaces(node_info
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)
target_link_libraries(node_info 
  ${LIBS} 
  ${PROJECT_NAME}_core
  ${rosidl_typesupport_cpp_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

add_executable(topic_info src/topic_info.cpp)
ament_target_dependencies(topic_info
  rcutils
  rclcpp
  rosidl_typesupport_cpp
)
rosidl_target_interfaces(topic_info
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)
target_link_libraries(topic_info 
  ${LIBS} 
  ${PROJECT_NAME}_core
  ${rosidl_typesupport_cpp_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

add_library(${PROJECT_NAME}_core SHARED
  src/logging.cpp
  src/node.cpp
  src/time.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  ${PROJECT_NAME}
  rclcpp
  rmw_fastrtps_cpp
  rosgraph_msgs
)
target_link_libraries(${PROJECT_NAME}_core ${LIBS})

if(BUILD_TESTING)
  #find_package(rostest REQUIRED)
  #add_rostest_gtest(test_params test/params.test test/params.cpp)
  #target_link_libraries(test_params ${LIBS})
endif()

### Install Test Node and Headers ###
install(TARGETS 
  ${PROJECT_NAME}_core
  nodelet_manager
  nodelet
  node_info
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
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()
