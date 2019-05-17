// C++ STL
#include <vector> // vector
#include <algorithm> // find
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
#include <pcl/common/transforms.h> // Transform pointcloud
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/registration/ndt.h> // NDT
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
#include <pcl/filters/extract_indices.h> // Indice filter
// MSG
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;

class hw6{
 private:
  bool use_icp; // Registration method, use ICP if 1, NDT otherwise
  /* 
   * Process status
   * -1: fail
   *  0: still processing
   *  1: complete
   */
  int status; 
  int count; // index counter
  const int NUMOFSCAN = 179; // number of scan, which is a constant
  const std::vector<int> step_to_add; // which index will publish map pc, constant
  double leaf_size; // leaf size for voxel grid, from parameter server
  double plane_thin; // plane distance threshold for model fitting, from parameter server
  double lower_z; // z lower bound for pass through filter, from parameter server
  double upper_z; // z upper bound for pass through filter, from parameter server
  std::string package_path; // path for package
  std::string file_path; // file_path = package_path + "/pcd/" + \count\ + ".pcd"
  PointXYZ map; // Pointcloud to publish as map
  PointXYZ input; // Input pointcloud, read from pcd file
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // ICP object
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt; // NDT object
  Eigen::Matrix4f guess; // Initial guess for registration
  ros::NodeHandle nh_; // public node Handler
  ros::NodeHandle pnh_; // private node handle
  ros::Publisher pub_map; // Publisher
  /*
   *  Post processing for map point cloud
   *  Since velodyne point cloud includes rings, which snarl up the scene
   *  Process:
   *    1. Use RANSAC to find plane inliers
   *    2. Then remove these points in map point cloud
   *  Plane coefficients and number of inlier points are also provided
   */
  void postprocessing(void);
  /*
   *  Publish map point cloud
   *  Process:
   *    1. Convert pcl::PointCloud<pcl::PointXYZ> to pcl::PCLPointCloud2
   *    2. Then convert pcl::PCLPointCloud2 to sensor_msgs::PointCloud2 by pcl_conversions
  *     3. Finally add header infomation and publish data
   */
  void publishMap(void);
  /*
   *  Process the pcd sequence
   *  Process:
   *    1. Read pcd file
   *    2. Do registration between map and input scan
   *    3. Publish map if needed
   *  [param]out bool: false if cannot read pcd file, true otherwise
   */
  bool start_process(void);
  /*
   *  Input scan preprocessing, including voxel grid downsampling and pass through filter
   *  [param]in PointXYZPtr pc: input scan pointer
   *  [param]out PointXYZ: point cloud after preprocessing
   */
  PointXYZ preprocessing(PointXYZPtr);
 public:
  // Constructor
  hw6(ros::NodeHandle nh, ros::NodeHandle pnh, std::vector<int> const& vec): 
    nh_(nh), pnh_(pnh), count(1), step_to_add(vec), status(0){
    package_path = ros::package::getPath("hw6_0751904");
    // Get parameters
    if(!pnh_.getParam("use_icp", use_icp)) use_icp = true; ROS_INFO("use_icp: %s", (use_icp==true?"true":"false"));
    if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1; ROS_INFO("leaf_size: %f", leaf_size);
    if(!pnh_.getParam("plane_thin", plane_thin)) plane_thin = 0.3; ROS_INFO("plane_thin: %f", plane_thin);
    if(!pnh_.getParam("lower_z", lower_z)) lower_z = -5.0; ROS_INFO("lower_z: %f", lower_z);
    if(!pnh_.getParam("upper_z", upper_z)) upper_z = 5.0; ROS_INFO("upper_z: %f", upper_z);
    pub_map = pnh_.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
    // Set initial guess to identity
    guess = Eigen::Matrix4f::Identity();
    //consecutive = Eigen::Matrix4f::Identity();
    // ICP converge criteria
    if(use_icp){
      icp.setMaximumIterations(150);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
    }else{
      ndt.setMaximumIterations(150);
      ndt.setTransformationEpsilon(1e-6);
      ndt.setEuclideanFitnessEpsilon(1e-6);
      ndt.setStepSize(0.1); // XXX: what this parameter means?
      ndt.setResolution(1.0); // XXX: what this parameter means?
    }
    if(!start_process()) status = -1;
    std::string save_path = package_path + "/map.pcd";
    ROS_INFO("Before processing, there are %d points in the map", (int)map.points.size());
    ROS_INFO("Postprocessing..."); postprocessing();
    publishMap();
    if(map.points.size() != 0){
      pcl::io::savePCDFileASCII(save_path, map);
      ROS_INFO("File: %s saved, there are %d points, existing...", save_path.c_str(), (int)map.points.size());
      status = 1; ROS_INFO("Complete, shutdown...");
    }
  }
  int getStatus(void) {return status;}
};

// ************************************************************************************************
// ********************************************* MAIN *********************************************
// ************************************************************************************************
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw6_node");
  ros::NodeHandle nh, pnh("~");
  std::vector<int> step_to_add{25, 49, 74, 99, 124, 151, 179};
  hw6 foo(nh, pnh, step_to_add);
  while(ros::ok()){
    ros::spinOnce(); if(foo.getStatus() != 0) ros::shutdown();
  }
  return 0;
}

void hw6::publishMap(void){
  // Publish scan
  sensor_msgs::PointCloud2 pcout;
  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(map, pc2); // First convert to pclpc2
  pcl_conversions::fromPCL(pc2, pcout);
  pcout.header.frame_id = "map"; // Add frame id
  pub_map.publish(pcout); // Then publish
}

bool hw6::start_process(void){
  while(count<=NUMOFSCAN){
    ROS_INFO("----------------- %d -----------------", count);
    file_path = package_path + "/pcd/" + std::to_string(count) + ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, input) == -1){
      ROS_ERROR("Cannot find index %d, aborting...", count); return false;
    }
    if(count==1) { // First data
      map += input; ++count; 
      publishMap();
      continue;
    }
    // Do registration
    PointXYZ filtered_pc = preprocessing(input.makeShared());
    ROS_INFO("Scan %d | %d -> %d", count, (int)input.points.size(), (int)filtered_pc.points.size());
    PointXYZ transformed_pc;
    ros::Time now = ros::Time::now();
    if(use_icp){
      icp.setInputSource(filtered_pc.makeShared());
      icp.setInputTarget(map.makeShared());
      icp.align(transformed_pc, guess);
      ROS_INFO("ICP time: %f", (ros::Time::now() - now).toSec());
      // Update initial guess for nect input
      guess = icp.getFinalTransformation(); 
    } else{
      ndt.setInputSource(filtered_pc.makeShared());
      ndt.setInputTarget(map.makeShared());
      ndt.align(transformed_pc, guess);
      // Update initial guess for nect input
      guess = ndt.getFinalTransformation(); 
    }
    // Update map if count in step_to_add vector
    std::vector<int>::const_iterator it;
    it = find(step_to_add.begin(), step_to_add.end(), count);
    if(it != step_to_add.end()){ // Found
      pcl::transformPointCloud (input, transformed_pc, guess);
      map += transformed_pc;
      ROS_INFO("%d points add to map", (int)transformed_pc.points.size());
      // Publish 
      publishMap();
    } ++count;
  } // End while
  return true;
}

PointXYZ hw6::preprocessing(PointXYZPtr pc){
  PointXYZ out;
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(out);
  // Pass through filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(out.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (lower_z, upper_z);
  pass.filter(out);
  return out;
}

void hw6::postprocessing(void){
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
  // Optional
  seg_plane.setOptimizeCoefficients(true);
  // Mandatory
  seg_plane.setModelType (pcl::SACMODEL_PLANE);
  seg_plane.setMethodType (pcl::SAC_RANSAC);
  seg_plane.setDistanceThreshold(plane_thin);
  
  seg_plane.setInputCloud (map.makeShared());
  seg_plane.segment (*inliers, *coefficients);
  
  ROS_INFO("There are %d ground inliers\n \
            Ground coefficeints: [%f] x + [%f] y + [%f] z = [%f]",
            (int)inliers->indices.size(),
            coefficients->values[0], coefficients->values[1], 
            coefficients->values[2], coefficients->values[3]*-1);

  pcl::ExtractIndices<pcl::PointXYZ> indiceFilter(true); // Initializing with true will allow us to extract the removed indices
  indiceFilter.setInputCloud(map.makeShared());
  indiceFilter.setIndices(inliers);
  indiceFilter.setNegative(true);
  indiceFilter.filter(map);
}
