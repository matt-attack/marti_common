cmake_minimum_required(VERSION 3.5)
project(swri_transform_util)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(gps_common REQUIRED)
find_package(swri_math_util REQUIRED)
#find_package(geographic_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(swri_roscpp REQUIRED)
find_package(swri_yaml_util REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(topic_tools REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(PROJ REQUIRED proj)

set(INCLUDE_DIRS include ${gps_common_INCLUDE_DIRS}
  ${swri_math_util_INCLUDE_DIRS} 
  #${geographic_msgs_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} 
  #${swri_roscpp_INCLUDE_DIRS}
  ${swri_yaml_util_INCLUDE_DIRS} ${diagnostic_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${topic_tools_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${gps_common_LIBRARY_DIRS} ${swri_math_util_LIBRARY_DIRS}
#  ${geographic_msgs_LIBRARY_DIRS} 
  ${rclpy_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  #${swri_roscpp_LIBRARY_DIRS} 
  ${swri_yaml_util_LIBRARY_DIRS}
  ${diagnostic_msgs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS} 
  #${topic_tools_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${gps_common_LIBRARIES} ${swri_math_util_LIBRARIES}
#  ${geographic_msgs_LIBRARIES} 
  ${rclpy_LIBRARIES} ${rclcpp_LIBRARIES}
  #${swri_roscpp_LIBRARIES} 
  ${swri_yaml_util_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES} ${tf2_LIBRARIES} ${topic_tools_LIBRARIES}
  ${ament_cmake_LIBRARIES} ${geometry_msgs_LIBRARIES})

find_package(OpenCV REQUIRED core highgui imgproc video)
find_package(Boost REQUIRED)

#generate_dynamic_reconfigure_options(cfg/DynamicPublisher.cfg)

add_library(${PROJECT_NAME} src/georeference.cpp 
  src/local_xy_util.cpp
  src/utm_util.cpp 
  src/transform.cpp 
  src/transformer.cpp 
  src/transform_manager.cpp
  src/transform_util.cpp 
  src/utm_transformer.cpp 
  src/wgs84_transformer.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${OpenCV_LIBS} ${PROJ_LIBRARIES} ${Boost_LIBRARIES})

#add_library(${PROJECT_NAME}_nodelets src/nodelets/dynamic_publisher.cpp)
#target_link_libraries(${PROJECT_NAME}_nodelets ${LIBS})
#add_dependencies(${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gencfg)

#add_executable(lat_lon_tf_echo src/nodes/lat_lon_tf_echo.cpp)
#target_link_libraries(lat_lon_tf_echo ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_local_xy_util launch/local_xy_util.test
    test/test_local_xy_util.cpp)
  target_link_libraries(test_local_xy_util ${PROJECT_NAME})

  add_rostest_gtest(test_utm_util launch/utm_util.test test/test_utm_util.cpp)
  target_link_libraries(test_utm_util ${PROJECT_NAME})

  add_rostest_gtest(test_transform_manager launch/transform_manager.test
    test/test_transform_manager.cpp)
  target_link_libraries(test_transform_manager ${PROJECT_NAME})

  add_rostest_gtest(test_georeference launch/georeference.test
    test/test_georeference.cpp)
  target_link_libraries(test_georeference ${PROJECT_NAME})

  add_rostest_gtest(test_transform_util launch/transform_util.test
    test/test_transform_util.cpp)
  target_link_libraries(test_transform_util ${PROJECT_NAME})

  if($ENV{ROS_DISTRO} STREQUAL "indigo")
    add_rostest(test/initialize_origin_auto_gps.test)
  else() # Jade, Kinetic, Lunar, etc
    add_rostest(test/initialize_origin_auto_navsat.test)
  endif()
  add_rostest(test/initialize_origin_manual.test)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

install(TARGETS ${PROJECT_NAME} 
  #${PROJECT_NAME}_nodelets #lat_lon_tf_echo
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib
  ARCHIVE
  DESTINATION lib)

### Install Python Nodes/Scripts ###
#catkin_install_python(PROGRAMS nodes/initialize_origin.py
#  DESTINATION lib/${PROJECT_NAME})

install(FILES nodelet_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(gps_common)
ament_export_dependencies(swri_math_util)
#ament_export_dependencies(geographic_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(rclcpp)
#ament_export_dependencies(swri_roscpp)
ament_export_dependencies(swri_yaml_util)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
#ament_export_dependencies(topic_tools)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_nodelets ${LIBS} ${Boost_LIBRARIES} ${PROJ_LIBRARIES})

ament_package()
