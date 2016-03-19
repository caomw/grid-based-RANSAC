cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(seg)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
include
)

link_directories(
${PCL_LIBRARY_DIRS}
)


add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (grid_based_ransac src/grid_based_ransac.cpp src/segmentplane.cpp src/subplane.cpp src/tool.cpp)
target_link_libraries (grid_based_ransac ${PCL_LIBRARIES} ${OpenCV_LIBS})
