// C++
#include <thread>
#include <algorithm> // find
// ROS
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
// MSG
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h> // Transform pointcloud
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/extract_indices.h> // Indice filter
#include <pcl/segmentation/extract_clusters.h> // Euclidean Cluster Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
#include <pcl/registration/icp.h>
// OpenCV
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>
// MessageFilter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Sleep, just for debug
#include <unistd.h>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace ros;

typedef PointCloud<PointXYZI> PointCloudXYZI;
typedef PointCloud<PointXYZI>::Ptr PointCloudXYZIPtr;
// Data Structure
typedef vector<PointCloudXYZI> ClusterVector;
typedef vector<Vector4f> CentroidVector;
typedef tuple<ros::Time, CentroidVector, ClusterVector, PointCloudXYZIPtr> DataTuple;
typedef vector<DataTuple> DataTupleVector;
typedef tuple<ros::Time, int, Vector4f, PointCloudXYZI> ResultTuple;
typedef vector<ResultTuple> ResultVector;
typedef vector<ResultVector> ResultVectors;

class Track{
 private:
  // Variables
  bool status = false;
  bool firstProcess = true;
  bool startReceive = false;
  bool bagHasEnd = false;
  const int WIDTH = 1280, HEIGHT = 720;
  int frame_process_count;
  double leaf_size; // voxelgrid size, from parameter server
  double clusterDistThres; // distance threshold from cluster to center
  double centroidDistThres; // distance threshod between centroid
  double clusterTolerance;
  double lower_z;
  Time startReceiveTime;
  NodeHandle nh_, pnh_;
  DataTupleVector TupleVector;
  ResultVectors RVector;
  Publisher result_pub, filtered_pub, cluster_pub;
  Subscriber lidar_sub;

  visualization_msgs::MarkerArray markerArray;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_current_frame;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_last_frame;

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

  void cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg);
 public:
  Track(ros::NodeHandle nh, ros::NodeHandle pnh);
  bool getStatus(void) {return status;}
  void process_data(void);
  void check_bag_has_end(void);
  void write_result(void);
  void plot_marker(void);
};
