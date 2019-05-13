#include <iostream>
#include <cstdlib>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

void preprocessing(pcl::PointCloud<pcl::PointXYZ>&);

int
 main (int argc, char** argv)
{
  //std::string map_string = "/home/sean/Downloads/SDC/test/src/map.pcd";
  std::string scan_string = "/home/sean/Downloads/SDC/test/src/scan/scan_";
  std::string extension = ".pcd";
  int count = 0;
  //pcl::PointCloud<pcl::PointXYZ> map;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::PointCloud<pcl::PointXYZRGB> first_scan_color;
  // Load pcd files
  /*if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_string, map) == -1){
    std::cout << "Fail to load map." << std::endl;
    return -1;
  }*/
  while(count<48){
    std::string file = scan_string + std::to_string(count) + extension;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file, scan) == -1){
      std::cout << "Fail to load map." << std::endl;
      return -1;
    }
    // preprocessing
    //preprocessing(map); std::cout << "Map: " << map.points.size() << " points\n";
    preprocessing(scan); std::cout << "Scan: " << scan.points.size() << " points\n";
    pcl::visualization::PCLVisualizer viewer(std::to_string(count));
    viewer.addCoordinateSystem(10.0);
    //viewer.addPointCloud(map.makeShared(), "map");
    viewer.addPointCloud(scan.makeShared(), std::to_string(count));
    //while(!viewer.wasStopped())
    viewer.spinOnce(500);
    viewer.removeAllPointClouds();
    viewer.close();
    count++;
  }
  /*pcl::copyPointCloud(first_scan, first_scan_color);
  for(auto it=first_scan_color.points.begin(); it!=first_scan_color.points.end(); ++it)
      it->r = 255;
  
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  float theta = 145.0 * M_PI/180.0; // The angle of rotation in radians
  guess(0,0) = cos (theta);
  guess(0,1) = -sin(theta);
  guess(1,0) = sin (theta);
  guess(1,1) = cos (theta);
  guess(0,3) = -2.0; 
  guess(1,3) = 8.0;
  guess(2,3) = -3.0;*/
  /*
  pcl::PointCloud<pcl::PointXYZRGB> temp;
  pcl::transformPointCloud(first_scan_color,temp, guess);
  pcl::visualization::PCLVisualizer viewer("ICP");
  viewer.addCoordinateSystem(10.0);
  viewer.addPointCloud(map.makeShared(), "map");
  viewer.addPointCloud(temp.makeShared(), "transformed");
  while(!viewer.wasStopped())
    viewer.spinOnce();
  */
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(25);
  
  /*while(1){
    clock_t ts = clock();
    icp.setInputSource(first_scan.makeShared());
    icp.setInputTarget(map.makeShared());
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final, guess);
    std::cout << "ICP calculation time: " << (double)((clock() - ts)/CLOCKS_PER_SEC) << "\n";
    std::cout << icp.getFinalTransformation() << std::endl;
    std::cout << "Maximum iteration: "<< icp.getMaximumIterations() << "\n";
    std::cout << "RANSAC iteration: "<< icp.getRANSACIterations() << "\n";
    std::cout << "RANSAC outlier rejection threshold: "<< icp.getRANSACOutlierRejectionThreshold() << "\n";
    std::cout << "Maximum Euclidean Fitness Epsilon : "<< icp.getEuclideanFitnessEpsilon() << "\n";
    guess = icp.getFinalTransformation();
    pcl::PointCloud<pcl::PointXYZRGB> Final_color;
    pcl::copyPointCloud(Final, Final_color);
    // Add color
    for(auto it=Final_color.points.begin(); it!=Final_color.points.end(); ++it){
      it->r = 255;
    }
    // Vis
    pcl::visualization::PCLVisualizer viewer("ICP");
    viewer.addPointCloud(map.makeShared(), "map");
    viewer.addPointCloud(Final_color.makeShared(), "transformed");
    while(!viewer.wasStopped())
      viewer.spinOnce();
  }*/
  return (0);
}

void preprocessing(pcl::PointCloud<pcl::PointXYZ> &p){
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(p.makeShared());
  vg.setLeafSize (0.09f, 0.09f, 0.09f);
  vg.filter(p);
  pcl::PointCloud<pcl::PointXYZ> temp;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(p.makeShared());
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  pcl::PointXYZ origin(0.0, 0.0, 0.0);
  if(kdtree.radiusSearch(origin, 18, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
    for(size_t i=0; i<pointIdxRadiusSearch.size(); ++i){
      temp.points.push_back(p.points[pointIdxRadiusSearch[i]]);
    }
    pcl::copyPointCloud(temp, p);
  }
}
