cmake_minimum_required(VERSION 3.5)
project(swri_yaml_util)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${swri_yaml_util_BINARY_DIR}/include)
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
message(STATUS "YAML-CPP VERSION: " ${YamlCpp_VERSION})
message(STATUS "YAML-CPP LIBRARY: " ${YamlCpp_LIBRARIES})

file(MAKE_DIRECTORY include/${PROJECT_NAME})

if(${YamlCpp_VERSION} MATCHES "0.[23].*")
  set(YAMLCPP_OLD_API ON)
  message(STATUS "  Using old yaml-cpp API.")
else()
  set(YAMLCPP_OLD_API OFF)
  message(STATUS "  Using new yaml-cpp API.")
endif()

configure_file(version.h.in
  include/${PROJECT_NAME}/version.h)

message(STATUS "Path: include/" ${PROJECT_NAME})

find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED src/yaml_util.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${YamlCpp_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  RUNTIME
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib
  ARCHIVE
  DESTINATION lib)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS} ${YamlCpp_LIBRARIES})

ament_package()
