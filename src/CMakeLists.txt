cmake_minimum_required(VERSION 3.16)
 
project(PointCloudProject)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
 
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(generatePointCloud generatePointCloud.cpp )
target_link_libraries(generatePointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
