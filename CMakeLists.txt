cmake_minimum_required(VERSION 3.0.2)
project(fpr)


SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 ") #14

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
message(STATUS "TBB version: ${TBB_VERSION}")
find_package(OpenCV REQUIRED)

find_package(tsl-robin-map QUIET NO_MODULE)
include(FetchContent)
FetchContent_Declare(tessil SYSTEM URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.0.1.tar.gz)
FetchContent_MakeAvailable(tessil)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy 
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS  
)

 
SET(TARGET_LINK_LIBS ${catkin_LIBRARIES} ${PCL_LIBRARIES} TBB::tbb ${OpenCV_LIBRARIES} tsl::robin_map)
SET(MAIN_SOURCE_FILES src/main.cpp)


add_executable(main ${MAIN_SOURCE_FILES}) 
target_link_libraries(main ${TARGET_LINK_LIBS}) 
add_dependencies(main ${catkin_EXPORTED_TARGETS})











