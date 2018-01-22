cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(narf_feature_extraction)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp_narf icp_narg.cpp)
target_link_libraries (icp_narf ${PCL_LIBRARIES})

add_executable(icp_narf_one_file_output icp_narf_one_file_output.cpp)
target_link_libraries (icp_narf_one_file_output ${PCL_LIBRARIES})