cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PointCloudBrowse)


#OpenCv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Eigen
find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Browse main.cpp)

target_link_libraries(Browse ${PCL_LIBRARIES} ${OpenCV_LIBS})