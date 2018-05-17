cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(NAVI)

set (CMAKE_CXX_STANDARD 11)
set (EIGENPATH "/home/baxter/Program/eigen/" )

find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGENPATH} )
include_directories(${OpenCV_INCLUDE_DIRS} )


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(run main.cpp)

target_link_libraries(run ${OpenCV_LIBS}  ${PCL_LIBRARIES})