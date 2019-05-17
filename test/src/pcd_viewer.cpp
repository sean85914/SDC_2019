#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

extern bool type; // 0 -> xyz, 1 -> rgbxyz

void showHelp(char* program){
  std::string str(program);
  std::cout << "\033[1;31mNot enough input, please provide pcd file...\033[0m\n"
            << "Program name: \n\t" << str.substr(2, str.length()) << "\n";
  if(type==0)
    std::cout << "Usage: \n\tvisualize the pcd file you provide, which include only XYZ data\n";
  else
    std::cout << "Usage: \n\tvisualize the pcd file you provide, which include XYZ and RGB data\n";
}
int main(int argc, char** argv)
{
  if(argc!=2){
    showHelp(argv[0]);
    return -1;
  }
  if(type==0){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1){
      std::cout << "Cannot load provided pcd file!\n";
      return -1;
    }
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {}
  }
  if(type==1){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1){
      std::cout << "Cannot load provided pcd file!\n";
      return -1;
    }
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {}
  }
  return 0;
}
