/*
 *  ICP SLAM
 *  Use consecutive scan to build the map of environment
 *
 *  If offline set to true. the node will try to find the scan and build the map
 *  If offline set to false, the node will subscribe to pointcloud topic and build the map
 *  
 *  Editor: Sean Lu (sean19960914@gmail.com)
 *  Last Edited: 5/24, 2019
 *  ROS related:
 *
 *    Publisher:
 *      ~map_pc(sensor_msgs::PointCloud2): pointcloud of map
 *      ~path(nav_msgs::Path): path of the robot navigated
 *    Subscriber:
 *      ~velodyne_points(sensor_msgs::PointCloud2): subscribe if offline set to false
 *    Parameters:
 *      ~removeGround: whether remove dominant ground after map built
 *      ~use_icp: whether use icp, true for ICP and false for NDT
 *        KNOWN_ISSUE: NDT takes too much time, not recommend to use false
 *      ~numofscan: if offline, the number of scan should be provided
 *      ~saveEvery: after how much frame should add scan to map
 *      ~have_odom: whether use wheel odometry information as initial guess for registration
 *      ~leaf_size: leaf size for voxel grid pointcloud downsampling
 *      ~plane_thin: use if removeGround set to true, the thickness of the plane
 *      ~lower_z: lower range of z for pass though filter, z value less than this will be neglected
 *      ~upper_z: upper range of z for pass though filter, z value greater than this will be neglected
 *      ~lower_r: lower range for distance filter, point with distance less than this will be neglected
 *      ~upper_r: upper range for distance filter, point with distance greater than this will be neglected
 *      ~prefix: use if offline set to true, prefix for pcd file
 *         pcd file should be formatted as: \prefix\ + "_" + 1 ~ \numofscan\ + ".pcd"
 *      ~len_of_vec: use if offline set to false, batch size for pointcloud use as registration target
 *      ~folfer: pcd file folder
 *         pcd file should be placed in \package_path\ + "/" + \folder\
 */

#ifndef ICP_SLAM_H_
#define ICP_SLAM_H_
// C++ STL
#include <cassert> // assert
#include <fstream> // ifstream
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ, Normal. PointNormal 
#include <pcl/common/transforms.h> // Transform pointcloud
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/registration/ndt.h> // NDT
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/features/normal_3d.h> // Compute normal
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
typedef pcl::PointCloud<pcl::Normal> Normal;
typedef pcl::PointCloud<pcl::PointNormal> PointNormal;
typedef std::vector<PointXYZ> pc_vector;

class ICPSLAM{
 private:
  bool removeGround; // Whether remove ground in map, from parameter server
  bool have_odom; // Whether if have wheel odometry, from parameter server
  bool use_icp; // Registration method, use ICP if 1, NDT otherwise
  bool point2point; 
  bool offline_; // Whether use saved pcd as input, from constructor
  /* 
   * Process status
   * -1: fail
   *  0: still processing
   *  1: complete
   */
  int status; 
  int len_of_vec; // length of pointcloud vector, from parameter server
  int mapDownsamplingCounter; // Counter to count for map downsampling
  int count; // index counter
  int numofscan; // number of scan, from parameter server
  int saveEvery; // save map pointcloud file every this integer, from parameter server
  double leaf_size; // leaf size for voxel grid, from parameter server
  double plane_thin; // plane distance threshold for model fitting, from parameter server
  double lower_z; // z lower bound for pass through filter, from parameter server
  double upper_z; // z upper bound for pass through filter, from parameter server
  double lower_r; // Lower bound for radius serach, from parameter server
  double upper_r; // Upper bound for radius serach, from parameter server
  double dx, dy;
  std::string prefix; // pcd file prefix, from parameter server
  std::string folder; // pcd file should be placed in this folder
  std::string package_path; // path for package
  std::string file_path; // file_path = package_path + "/" + \folder\ + "/" + \count\ + ".pcd"
  std::ifstream fin; // input file that contain wheel odometry information
  OdomData origin; // First recorded data
  PointXYZ map; // Pointcloud to publish as map
  PointXYZ input; // Input pointcloud, read from pcd file
  PointXYZ input_filtered; // Input pointcloud after filter
  pc_vector pc_v; // vector for pointcloud
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // ICP object
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_plane; // ICP point to plane object
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt; // NDT object
  Eigen::Matrix4f guess; // Initial guess for registration
  Eigen::Matrix4f last_transform;
  Eigen::Matrix3f odomTransform; 
  //Eigen::Matrix4f past; // Last transformation before ICP update
  ros::NodeHandle nh_; // public node Handler
  ros::NodeHandle pnh_; // private node handle
  // Publisher
  ros::Publisher pub_map; 
  ros::Publisher pub_path;
  nav_msgs::Path path;
  // Subscriber
  ros::Subscriber sub_pc;
  // Time stamp
  ros::Time last;
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
  /*
   *  Map downsampling to prevent too huge map
   */
  void mapDownsampling(void);
  /*
   *  Callback for pointcloud subscriber
   */
  void cb_pointcloud(const sensor_msgs::PointCloud2ConstPtr);
  /*
   *  Save map in \package_path\ + "/map.pcd"
   */
  void saveMap(void);
 public:
  // Constructor
  ICPSLAM(ros::NodeHandle, ros::NodeHandle, bool);
  // Custom shotdown process
  void mySigintHandler(int);
  // Returen status for offline process
  int getStatus(void) {return status;}
};
#endif
