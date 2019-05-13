cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_compile_options(-std=c++11)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Boost_DEFINITION})

#add_executable (iterative_closest_point src/test.cpp)
#target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable(test_fs src/test_fs.cpp)
target_link_libraries(test_fs ${Boost_LIBRARIES} -lboost_system -lboost_filesystem)
