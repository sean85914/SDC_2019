// C++
#include <utility> // pair
#include <cassert> // assert
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
#include <pcl/registration/icp.h> // ICP
#include <pcl/segmentation/extract_clusters.h> // Euclidean Cluster Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
// OpenCV
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>
// MessageFilter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef std::vector<Eigen::Vector4f> vector_of_centroid;
typedef std::vector<vector_of_centroid> vector_of_vectors;

class Track{
 private:
  // Variables
  bool firstProcess = true;
  int frame_process_count;
  const int WIDTH = 1280, HEIGHT = 720;
  const double distThres = 0.01;
  double leaf_size; // voxelgrid size, from parameter server
  double clusterTolerance;
  double lower_z;
  const double MAX_HEIGHT = 0.2f;
  const double MAX_DIS = 15.0f;
  const double MIN_DIS = 2.0f;
  ros::NodeHandle nh_, pnh_;
  vector_of_vectors vovs;
  sensor_msgs::PointCloud2 last_frame;
  ros::Publisher result_pub, filtered_pub, cluster_pub;
  ros::Subscriber lidar_sub;

  visualization_msgs::MarkerArray markerArray;
  std::vector<pcl::PointIndices> cluster_indices_last, cluster_indices;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inliers;
  void initMatrix(void);
  void pointCloudPreprocessing(void);
  void cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg);
 public:
  Track(ros::NodeHandle nh, ros::NodeHandle pnh);
};
