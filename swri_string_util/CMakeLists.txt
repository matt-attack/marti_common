cmake_minimum_required(VERSION 3.5)

project(swri_string_util)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

add_library(${PROJECT_NAME} SHARED src/string_util.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS})

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_string_util test/string_util.test
    test/test_string_util.cpp)
  target_link_libraries(test_string_util ${PROJECT_NAME})
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
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
