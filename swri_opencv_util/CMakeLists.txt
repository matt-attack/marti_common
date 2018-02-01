cmake_minimum_required(VERSION 3.5)
project(swri_opencv_util)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(swri_math_util REQUIRED)

set(INCLUDE_DIRS include ${cv_bridge_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${swri_math_util_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${cv_bridge_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${swri_math_util_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${cv_bridge_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${swri_math_util_LIBRARIES})

find_package(OpenCV REQUIRED core imgproc highgui)

# The Boost Random library headers and namespaces changed between version
# 1.46 and 1.47
find_package(Boost REQUIRED COMPONENTS serialization thread)
if(${Boost_LIB_VERSION} MATCHES "1_46*")
  add_definitions(-DBOOST_1_46)
endif(${Boost_LIB_VERSION} MATCHES "1_46*")

add_library(${PROJECT_NAME} src/models.cpp src/model_fit.cpp src/show.cpp
  src/convert.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${Boost_LIBRARIES} ${OpenCV_LIBS})

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

ament_export_dependencies(cv_bridge)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(swri_math_util)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
