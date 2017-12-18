cmake_minimum_required(VERSION 3.5)
project(swri_math_util)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES})

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost COMPONENTS random)
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions(-DBOOST_1_46)
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

add_library(${PROJECT_NAME} SHARED
  src/math_util.cpp src/trig_util.cpp src/random.cpp
#  src/interpolation_1d.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_trig_util launch/trig_util.test test/test_trig_util.cpp)
  target_link_libraries(test_trig_util ${PROJECT_NAME})

  add_rostest_gtest(test_math_util launch/math_util.test test/test_math_util.cpp)
  target_link_libraries(test_math_util ${PROJECT_NAME})

  add_rostest_gtest(test_random launch/random.test test/test_random.cpp)
  target_link_libraries(test_random ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib
  ARCHIVE
  DESTINATION lib)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
