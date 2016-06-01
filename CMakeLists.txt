cmake_minimum_required(VERSION 2.8.3)
project(dns_pattern_formation)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
  DNSPatternFormation 
  src/DynamicNeighbourSelection.cpp 
  src/Controller.h 
  src/Vector2d.h 
)	 

target_link_libraries(DNSPatternFormation ${catkin_LIBRARIES})



