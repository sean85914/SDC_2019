// C++ STL
#include <cstdlib> // itoa
#include <thread> // Support since C++11
// Boost
#include <boost/filesystem.hpp> // Support in STL since C++17
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/filters/passthrough.h> // passThrough
// TF
#include <tf/transform_broadcaster.h>
// MSG
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZ_ptr;

const double RATE = 3.0; // Frequency to publish map pc

class hw5{
 private:
  // ************************************************************************************************
  // ******************************************* Variable *******************************************
  // ************************************************************************************************
  bool save_pc; // Whether if save point cloud, from parameter server
  bool draw_path; // Whether if publish path message
  int count; // Frame counter
  double freq_; // Frequency to publish map pc, from constructor
  double leaf_size; // Leaf size for voxel grid downsampling, from parameter server
  double radius; // Radius inlier for ICP search, from parameter server
  double lower_z, upper_z; // Range for passThrough
  std::string package_path; // Package path string
  std::string pcd_path; // String to load pcd file
  ros::NodeHandle nh_; // Public node handler
  ros::NodeHandle pnh_; // Private node handler
  PointXYZ map_cloud; 
  // Subscriber and publisher
  ros::Subscriber sub_pc; 
  ros::Publisher pub_map_pc;
  ros::Publisher pub_scan_pc;
  ros::Publisher pub_path;
  nav_msgs::Path path;
  // ICP object
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  Eigen::Matrix4f guess; // Initial guess for ICP
  // ************************************************************************************************
  // ******************************************** Method ********************************************
  // ************************************************************************************************
  /*
   * Read map data, called in constructor
   * Parameters:
   *   [out] bool: 1 if success, 0 otherwise
  */
  bool read_map(void); 
  void cb_scan(const sensor_msgs::PointCloud2ConstPtr); // Callback for subscriber
  /*
   * Preprocessing for point cloud
   * Parameters:
   *   PointXYZ&: target point cloud reference
   *   bool: whether using distance filter, default is false
   *   bool: whether using pass through filter, default is false
   */
  void cloud_preprocessing(PointXYZ&, bool applyDistanceFilter = false, bool passThrough = false);
 public:
  // Constructor
  hw5(ros::NodeHandle nh, ros::NodeHandle pnh, double freq): 
    nh_(nh), pnh_(pnh), freq_(freq), count(0){
    package_path = ros::package::getPath("hw5_0751904");
    pcd_path = package_path + "/map.pcd";
    if(!read_map()){
      ROS_ERROR("Fail to load map, shutdown...");
      ros::shutdown();
    }
    //　Initial guess, after several tried
    guess = Eigen::Matrix4f::Identity(4, 4);
    float theta = 145.0*M_PI/180.0; // The angle of rotation in radians
    guess(0,0) = cos (theta);
    guess(0,1) = -sin(theta);
    guess(1,0) = sin (theta);
    guess(1,1) = cos (theta);
    guess(0,3) = -2.22595; 
    guess(1,3) = 7.70455;
    guess(2,3) = -2.9667;
    // ICP converge criteria
    icp.setMaximumIterations(150);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    // Get parameters
    if(!pnh_.getParam("save_pc", save_pc)) save_pc = false; ROS_INFO("save_pc: %s", (save_pc==true? "true": "false"));
    if(!pnh_.getParam("draw_path", draw_path)) draw_path = false; ROS_INFO("draw_path: %s", (draw_path==true? "true": "false"));
    if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.09; ROS_INFO("leaf_size: %f", leaf_size);
    if(!pnh_.getParam("radius", radius)) radius = 20.0; ROS_INFO("radius: %f", radius);
    if(!pnh_.getParam("lower_z", lower_z)) lower_z = -1.0; ROS_INFO("lower_z: %f", lower_z);
    if(!pnh_.getParam("upper_z", upper_z)) upper_z = 1.0; ROS_INFO("upper_z: %f", upper_z);
    if(draw_path){
      pub_path = pnh_.advertise<nav_msgs::Path>("path", 1);
      path.header.frame_id = "map";
    }
    // Map cloud preprocessing
    ROS_INFO("Success to load map!"); 
    ROS_INFO("Before preprocessing: %d points", map_cloud.height*map_cloud.width);
    cloud_preprocessing(map_cloud);
    ROS_INFO("After preprocessing: %d points", map_cloud.height*map_cloud.width);
    // Setup subscriber and publisher
    sub_pc = nh_.subscribe("/points_raw", 1, &hw5::cb_scan, this);
    pub_map_pc = pnh_.advertise<sensor_msgs::PointCloud2>("map", 1);
    pub_scan_pc = pnh_.advertise<sensor_msgs::PointCloud2>("scan", 1);
  }
   // Destructor
   ~hw5(){
     // Shutdown subscriber and publisher
     sub_pc.shutdown();
     pub_map_pc.shutdown();
     pub_scan_pc.shutdown();
     if(draw_path) pub_path.shutdown();
   }
   void thread_do(void); // Function for thread
};

// ************************************************************************************************
// ********************************************* MAIN *********************************************
// ************************************************************************************************
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw5_node");
  ros::NodeHandle nh, pnh("~");
  hw5 foo(nh, pnh, RATE);
  std::thread myThread(&hw5::thread_do, &foo);
  while(ros::ok()) ros::spinOnce();
  myThread.join(); // Wait until thread terminated
  return 0;
}

bool hw5::read_map(void){
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, map_cloud) == -1)
    return 0; // Fail
  return 1; // Success
}

void hw5::cb_scan(const sensor_msgs::PointCloud2ConstPtr msg){
  ROS_INFO("********    %d    ********", count);
  PointXYZ pc, final_pc;
  pcl::fromROSMsg(*msg, pc);
  cloud_preprocessing(pc, true, true);
  if(save_pc){
    // Check if "scan" folder exist
    boost::filesystem::path p(package_path+"/scan");
    if(!boost::filesystem::exists(p)){
      ROS_INFO("Directory doesnot exist, creating one...");
      boost::filesystem::create_directory(p);
    }
    std::string file_name = "/scan/scan_" + std::to_string(count) + ".pcd";
    pcl::io::savePCDFileASCII(package_path+file_name, pc);
  }
  // ICP
  icp.setInputSource(pc.makeShared());
  icp.setInputTarget(map_cloud.makeShared());
  ros::Time time = ros::Time::now();
  icp.align(final_pc, guess);
  ROS_INFO("ICP calculation time: %f second", (ros::Time::now() - time).toSec());
  guess = icp.getFinalTransformation(); 
  // Publish scan
  sensor_msgs::PointCloud2 pcout;
  pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(pc, pc2); // First convert to pclpc2
  pcl_conversions::fromPCL(pc2, pcout);
  pcout.header.frame_id = "scan"; // Add frame id
  pub_scan_pc.publish(pcout); // Then publish
  // Publish transform
  static tf::TransformBroadcaster br;
  tf::Matrix3x3 rot_mat(guess(0,0), guess(0,1), guess(0,2),
                        guess(1,0), guess(1,1), guess(1,2),
                        guess(2,0), guess(2,1), guess(2,2));
  tf::Quaternion quat; rot_mat.getRotation(quat); quat.normalize();
  tf::Vector3 trans(guess(0,3), guess(1,3), guess(2,3));
  tf::Transform transform(quat, trans);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scan"));
  ++count;
  if(draw_path){
    geometry_msgs::PoseStamped p;
    p.pose.position.x = guess(0,3);
    p.pose.position.y = guess(1,3);
    p.pose.position.z = guess(2,3);
    path.poses.push_back(p);
    pub_path.publish(path);
  }
}

void hw5::thread_do(void){
  ros::Rate r(freq_);
  while(ros::ok()){
    sensor_msgs::PointCloud2 pcout;
    pcl::PCLPointCloud2 pc2;
    pcl::toPCLPointCloud2(map_cloud, pc2); // First convert to pclpc2
    pcl_conversions::fromPCL(pc2, pcout);
    pcout.header.frame_id = "map"; // Add frame id
    pub_map_pc.publish(pcout); // Then publish
    r.sleep();
  }
}

void hw5::cloud_preprocessing(PointXYZ& pc, bool applyDistanceFilter, bool passThrough){
  // Remove nan
  std::vector<int> indice;
  pcl::removeNaNFromPointCloud(pc, pc, indice);
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pc.makeShared());
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(pc);
  // Distance filter
  if(applyDistanceFilter){
    PointXYZ temp; // Temp pc to save inliers
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pc.makeShared());
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ origin(0.0, 0.0, 0.0);
    if(kdtree.radiusSearch(origin, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
      for(size_t i=0; i<pointIdxRadiusSearch.size(); ++i){
        temp.points.push_back(pc.points[pointIdxRadiusSearch[i]]);
      }
      temp.width = (int)pointIdxRadiusSearch.size();
      temp.height = 1;
      pcl::copyPointCloud(temp, pc);
    }
  }
  // Pass through filter
  if(passThrough){
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (lower_z, upper_z);
    pass.filter (pc);
  }
}
