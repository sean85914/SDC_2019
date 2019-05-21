// C++ STL
#include <cassert> // assert
#include <fstream>
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef struct {
  double x;
  double y;
  double yaw;
}OdomData;
typedef pcl::PointCloud<pcl::PointXYZ> PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;

class hw6{
 private:
  bool removeGround; // Whether remove ground in map, from parameter server
  bool have_odom; // Whether if have wheel odometry, from parameter server
  bool use_icp; // Registration method, use ICP if 1, NDT otherwise
  /* 
   * Process status
   * -1: fail
   *  0: still processing
   *  1: complete
   */
  int status; 
  int count; // index counter
  int numofscan; // number of scan, from parameter server
  int saveEvery; // save map pointcloud file every this integer, from parameter server
  double leaf_size; // leaf size for voxel grid, from parameter server
  double plane_thin; // plane distance threshold for model fitting, from parameter server
  double lower_z; // z lower bound for pass through filter, from parameter server
  double upper_z; // z upper bound for pass through filter, from parameter server
  double lower_r; // Lower bound for radius serach, from parameter server
  double upper_r; // Upper bound for radius serach, from parameter server
  std::string prefix; // pcd file prefix, from parameter server
  std::string package_path; // path for package
  std::string file_path; // file_path = package_path + "/pcd/" + \count\ + ".pcd"
  std::ifstream fin; // input file that contain wheel odometry information
  OdomData origin; // First recorded data
  PointXYZ map; // Pointcloud to publish as map
  PointXYZ input; // Input pointcloud, read from pcd file
  PointXYZ input_filtered; // Input pointcloud after filter
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // ICP object
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt; // NDT object
  Eigen::Matrix4f guess; // Initial guess for registration
  //Eigen::Matrix3f odomTransform; 
  //Eigen::Matrix4f past; 
  ros::NodeHandle nh_; // public node Handler
  ros::NodeHandle pnh_; // private node handle
  // Publisher
  ros::Publisher pub_map; 
  ros::Publisher pub_path;
  nav_msgs::Path path;
  /*
   *  Post processing for map point cloud
   *  If removeGround set to true, will try to remove the dominant ground in map
   *  Process:
   *    1. Voxelgrid downsampling
   *    if removeGround, then
   *    2. Use RANSAC to find plane inliers
   *    3. Then remove these points in map point cloud
   *  Plane coefficients and number of inlier points are also provided
   *  [param]in removeGround: whether remove ground in map
   */
  void postprocessing(bool removeGround);
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
   *    3. Publish path and map if needed
   *  [param]out bool: false if cannot read pcd file or signal shutdown by user, true otherwise
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
  hw6(ros::NodeHandle nh, ros::NodeHandle pnh): 
    nh_(nh), pnh_(pnh), count(0) , status(0){
    package_path = ros::package::getPath("hw6_0751904");
    // Get parameters
    if(!pnh_.getParam("removeGround", removeGround)) removeGround = false; ROS_INFO("removeGround: %s", (removeGround==true?"true":"false"));
    if(!pnh_.getParam("use_icp", use_icp)) use_icp = true; ROS_INFO("use_icp: %s", (use_icp==true?"true":"false"));
    if(!pnh_.getParam("numofscan", numofscan)){
      ROS_ERROR("No numofscan provided, aborting...");
      status = -1; return;
    }
    if(!pnh_.getParam("have_odom", have_odom)) have_odom = false; ROS_INFO("have_odom: %s", (have_odom==true?"true":"false"));
    if(!pnh_.getParam("saveEvery", saveEvery)) saveEvery = 10; ROS_INFO("saveEvery: %d", saveEvery);
    if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1; ROS_INFO("leaf_size: %f", leaf_size);
    if(!pnh_.getParam("plane_thin", plane_thin)) plane_thin = 0.3; ROS_INFO("plane_thin: %f", plane_thin);
    if(!pnh_.getParam("lower_z", lower_z)) lower_z = -5.0; ROS_INFO("lower_z: %f", lower_z);
    if(!pnh_.getParam("upper_z", upper_z)) upper_z = 5.0; ROS_INFO("upper_z: %f", upper_z);
    if(!pnh_.getParam("lower_r", lower_r)) lower_r = 1.0; ROS_INFO("lower_r: %f", lower_r);
    if(!pnh_.getParam("upper_r", upper_r)) upper_r = 10.0; ROS_INFO("upper_r: %f", upper_r);
    if(!pnh_.getParam("prefix", prefix)) prefix=""; ROS_INFO("prefix: %s", prefix.c_str());
    assert(upper_z>lower_z); // Make sure upper_z > lower_z
    assert(lower_r>=0 and upper_r>=0); // Make sure nonnegative radius
    assert(upper_r>lower_r); // Maker sure upper_r > lower_r
    if(have_odom){
      fin.open("odometry.txt"); if(!fin) {ROS_ERROR("Cannot open file, aborting..."); status = -1; return;}
      std::string line;
      std::getline(fin, line);
      std::stringstream ss(line);
      double _; // redundant
      ss >> origin.x >> origin.y >> _ >> _ >> _ >> origin.yaw;
      ROS_INFO("Read first odometry data: %f, %f, %f", origin.x, origin.y, origin.yaw);
      /*odomTransform = Eigen::Matrix3f::Zero();
      odomTransform(0, 2) = origin.x; odomTransform(1, 2) = origin.y; odomTransform(2, 2) = 1.0;
      odomTransform(0, 0) = cos(origin.yaw); odomTransform(0, 1) = -sin(origin.yaw);
      odomTransform(1, 0) = sin(origin.yaw); odomTransform(1, 1) = cos(origin.yaw);*/
    }
    pub_map = pnh_.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
    pub_path = pnh_.advertise<nav_msgs::Path>("path", 1);
    // Inital path
    path.header.frame_id = "map";
    // Set initial guess to identity
    guess = Eigen::Matrix4f::Identity(); 
    //past = guess;
    // ICP converge criteria
    if(use_icp){
      icp.setMaximumIterations(300);
      icp.setTransformationEpsilon(1e-8);
      icp.setEuclideanFitnessEpsilon(1e-6);
    }else{
      ndt.setMaximumIterations(300);
      ndt.setTransformationEpsilon(1e-8);
      ndt.setEuclideanFitnessEpsilon(1e-6);
      ndt.setStepSize(0.1); // XXX: what this parameter means?
      ndt.setResolution(1.0); // XXX: what this parameter means?
    }
    if(!start_process()) status = -1;
    std::string save_path = package_path + "/map.pcd";
    postprocessing(removeGround);
    if(map.points.size() != 0){
      pcl::io::savePCDFileASCII(save_path, map);
      ROS_INFO("File: %s saved, there are %d points, existing...", save_path.c_str(), (int)map.points.size());
    }
    status = 1; ROS_INFO("Complete, shutdown...");
  }
  ~hw6(){
    fin.close(); pub_map.shutdown(); pub_path.shutdown();
  }
  int getStatus(void) {return status;}
};

// ************************************************************************************************
// ********************************************* MAIN *********************************************
// ************************************************************************************************
int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_slam");
  ros::NodeHandle nh, pnh("~");
  hw6 foo(nh, pnh);
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
  while(count<numofscan && ros::ok()){
    ROS_INFO("----------------- %d -----------------", count);
    file_path = package_path + "/pcd/" + (prefix==""?"":prefix+"_") + std::to_string(count) + ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, input) == -1){
      ROS_ERROR("Cannot find index %d, aborting...", count); return false;
    }
    if(count==0) { // First data
      map += input; ++count; 
      publishMap();
      geometry_msgs::PoseStamped p;
      path.poses.push_back(p);
      continue;
    }
    //past = guess;
    // Get odometry data and set as initial guess for ICP
    if(have_odom){
      std::string line;
      std::getline(fin, line);
      std::stringstream ss(line);
      OdomData odom;
      double _; // redundant
      ss >> odom.x >> odom.y >> _ >> _ >> _ >> odom.yaw;
      Eigen::Vector3f vec(odom.x - origin.x, odom.y - origin.y, 1.0);
      for(int i=0; i<2; ++i) guess(i, 3) += vec(i);
    }
    std::cout << guess << "\n";
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
    geometry_msgs::PoseStamped p;
    p.pose.position.x = guess(0, 3);
    p.pose.position.y = guess(1, 3);
    p.pose.position.z = guess(2, 3);
    path.poses.push_back(p);
    ROS_INFO("(%f, %f, %f)", guess(0, 3), guess(1, 3), guess(2, 3));
    pub_path.publish(path);
    // Update map if needed
    if(count%(saveEvery-1)==0){ 
      pcl::transformPointCloud(input_filtered, transformed_pc, guess);
      map += transformed_pc;
      ROS_INFO("%d points add to map, now there are %d points in map", (int)transformed_pc.points.size(), (int)map.points.size());
      // Publish 
      publishMap();
    } ++count;
    //for(int i=0; i<3; ++i) guess(i, 3) += guess(i, 3) - past(i, 3);
  } // End while
  if(count==numofscan) return true;
  else return false;
}

PointXYZ hw6::preprocessing(PointXYZPtr pc){
  PointXYZ out;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(out);
  // Distance filter using KD tree
  //   1. Distance larger than upper_r will be neglected
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ origin(0.0, 0.0, 0.0);
  std::vector<int> pointIdxRadiusSearch; // Vector to save inlier indices
  std::vector<float> pointRadiusSquaredDistance; // Vector to save corresponding distance
  if(upper_r>0){ // Equal 0 -> doesn;t have to do this
    kdtree.setInputCloud(out.makeShared());
    kdtree.radiusSearch(origin, upper_r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    extract.setInputCloud(out.makeShared());
    // Convert std::vector<int> to pcl::PointIndices
    boost::shared_ptr<std::vector<int>> indicesPtr (new std::vector<int> (pointIdxRadiusSearch));
    extract.setIndices(indicesPtr);
    extract.setNegative(false);
    extract.filter(out);
  }
  //   2. Distance smaller than lower_r will be neglected
  if(lower_r>0){ // Equal 0 -> doesn;t have to do this
    kdtree.setInputCloud(out.makeShared());
    // Clear container vector
    pointIdxRadiusSearch.clear(); pointRadiusSquaredDistance.clear();
    kdtree.radiusSearch(origin, lower_r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    extract.setInputCloud(out.makeShared());
    boost::shared_ptr<std::vector<int>> indicesPtr (new std::vector<int> (pointIdxRadiusSearch));
    extract.setIndices(indicesPtr);
    extract.setNegative(true); // Inliers will be erase
    extract.filter(out);
  }
  // Copy out to input_filter
  pcl::copyPointCloud(out, input_filtered);
  // Pass through filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(out.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (lower_z, upper_z);
  pass.filter(out);
  return out;
}

void hw6::postprocessing(bool removeGround){
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(map.makeShared());
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(map);
  if(removeGround){
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
}
