cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_compile_options(-std=c++17)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Boost_DEFINITION})

add_library(xyz_viewer include/xyz_viewer.cpp)
add_library(xyzrgb_viewer include/xyzrgb_viewer.cpp)

#add_executable (iterative_closest_point src/test.cpp)
#target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

#add_executable(test_fs src/test_fs.cpp)
#target_link_libraries(test_fs ${Boost_LIBRARIES} -lboost_system -lboost_filesystem)

add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries(normal_distributions_transform ${PCL_LIBRARIES})

add_executable(pcd_viewer_xyz src/pcd_viewer.cpp)
target_link_libraries(pcd_viewer_xyz ${PCL_LIBRARIES} xyz_viewer)

add_executable(pcd_viewer_xyzrgb src/pcd_viewer.cpp)
target_link_libraries(pcd_viewer_xyzrgb ${PCL_LIBRARIES} xyzrgb_viewer)

add_executable(icp_point_to_plane src/icp_point_to_plane.cpp)
target_link_libraries(icp_point_to_plane ${PCL_LIBRARIES})

add_executable(find_corner src/find_corner.cpp)
target_link_libraries(find_corner ${PCL_LIBRARIES})

install(TARGETS pcd_viewer_xyz pcd_viewer_xyzrgb
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  RUNTIME DESTINATION bin
)
