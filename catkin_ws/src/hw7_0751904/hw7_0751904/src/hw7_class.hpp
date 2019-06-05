// C++
#include <utility> // pair
#include <cassert> // assert
#include <algorithm> // find
// ROS
#include <ros/ros.h>
// Not real time, cannot work
/*#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
// MSG
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/common/transforms.h> // Transform pointcloud
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/extract_indices.h> // Indice filter
#include <pcl/segmentation/extract_clusters.h> // Euclidean Cluster Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
// OpenCV
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>

inline bool inRange(int data, int lowerRange, int upperRange){
  assert(lowerRange<upperRange);
  if(data>lowerRange and data<upperRange) return true;
  else return false;
}

class HW7{
 private:
  // Variables
  const int WIDTH = 1280, HEIGHT = 720;
  double size; // voxelgrid size, from parameter server
  double clusterTolerance;
  Eigen::Matrix4f extrinsic_matrix;
  cv::Mat intrinsic_matrix;
  cv::Mat distortion_coeff;
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_markerArray;
  ros::Subscriber sub_pc, sub_bbList;
  /*message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> sync;
  boost::shared_ptr<sync> sync_;*/
  visualization_msgs::MarkerArray markerArray;
  darknet_ros_msgs::BoundingBoxes bbList;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_removeGround;
  //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster;
  // Methods
  /*
   *  Initialize camera matrix and distortion vector
   */
  void initMatrix(void);
  /*
   *  Pointcloud preprocessing, including voxel grid downsampling, ground removal and clustering
   */
  void pointCloudPreprocessing(void);
  /*
   *  Callback for subscriber
   */
  //void callback(const sensor_msgs::PointCloud2ConstPtr &pc, const  darknet_ros_msgs::BoundingBoxesConstPtr &boxes);
  void cb_pc(const sensor_msgs::PointCloud2ConstPtr &pc);
  void cb_bbList(const darknet_ros_msgs::BoundingBoxesConstPtr &bbList);
 public:
  HW7(ros::NodeHandle nh, ros::NodeHandle pnh);
};

HW7::HW7(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
  intrinsic_matrix = cv::Mat(3, 3, cv::DataType<double>::type);
  distortion_coeff = cv::Mat(5, 1, cv::DataType<double>::type);
  initMatrix();
  cloud_raw = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_removeGround = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  //cloud_cluster = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  sub_pc = nh_.subscribe("/points_raw", 1, &HW7::cb_pc, this);
  sub_bbList = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &HW7::cb_bbList, this);
  pub_markerArray = pnh.advertise<visualization_msgs::MarkerArray>("text", 1);
  if(!pnh_.getParam("size", size)) size = 0.05f;
  if(!pnh_.getParam("clusterTolerance", clusterTolerance)) clusterTolerance = 1.0f;
  /*cloud_sub.subscribe(nh_, "/points_raw", 1);
  bb_sub.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
  sync_.reset(new sync(MySyncPolicy(1), cloud_sub, bb_sub));
  sync_->registerCallback(boost::bind(&HW7::callback, this, _1, _2));*/
  //ROS_INFO("[%s] Node ready!", ros::this_node::getName().c_str());
}

void HW7::initMatrix(void){
  extrinsic_matrix <<  0.84592974185943604,   0.53328412771224976, -0.0033089336939156055, 0.092240132391452789,
                      0.045996580272912979, -0.079141519963741302,   -0.99580162763595581, -0.35709697008132935, 
                      -0.53130710124969482,   0.84222602844238281,  -0.091477409005165100, -0.16055910289287567,
                                         0,                     0,                      0,                    1;
  intrinsic_matrix.at<double>(0, 0) = 698.939;
  intrinsic_matrix.at<double>(1, 0) = 0;
  intrinsic_matrix.at<double>(2, 0) = 0;

  intrinsic_matrix.at<double>(0, 1) = 0;
  intrinsic_matrix.at<double>(1, 1) = 698.939;
  intrinsic_matrix.at<double>(2, 1) = 0;

  intrinsic_matrix.at<double>(0, 2) = 641.868;
  intrinsic_matrix.at<double>(1, 2) = 385.788;
  intrinsic_matrix.at<double>(2, 2) = 1.0;

  distortion_coeff.at<double>(0) = -0.171466;
  distortion_coeff.at<double>(1) = 0.0246144;
  distortion_coeff.at<double>(2) = 0;
  distortion_coeff.at<double>(3) = 0;
  distortion_coeff.at<double>(4) = 0;
}

void HW7::pointCloudPreprocessing(void){
  //ROS_INFO("Start pointcloud preprocessing...");
  // Clear old vector
  cluster_indices.clear();
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_raw);
  vg.setLeafSize (size, size, size);
  vg.filter(*cloud_filtered);
  // Ground removal
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.05);
  // Extract the result
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_removeGround);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_removeGround);
  // Euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance); // 1 meter
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(1500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //ROS_INFO("Cluster size: %d", (int)cluster_indices.size());
}

/*
void HW7::callback(const sensor_msgs::PointCloud2ConstPtr &pc, const darknet_ros_msgs::BoundingBoxesConstPtr &boxes){
  pcl::fromROSMsg(*pc, *cloud_raw);
  pointCloudPreprocessing();
}*/


void HW7::cb_pc(const sensor_msgs::PointCloud2ConstPtr &pc){
  pcl::fromROSMsg(*pc, *cloud_raw);
  // clear marker array
  markerArray.markers.clear();
  pointCloudPreprocessing();
  int id_count = 0;
  std::vector<int> indexList;
  for(auto y: cluster_indices){
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
   for(auto z: y.indices){
     cloud_cluster->points.push_back(cloud_filtered->points[z]);
   }
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*cloud_cluster, centroid);
   double dist = std::sqrt(centroid(0)*centroid(0) + centroid(1)*centroid(1) + centroid(2)*centroid(2));
   if(dist > 50.0f) continue; // Too far away, neglect
   Eigen::Vector4f centroid_in_cam_coord = extrinsic_matrix * centroid;
   cv::Point3f centroid_in_cam_coord3f(centroid_in_cam_coord(0), centroid_in_cam_coord(1), centroid_in_cam_coord(2));
   double coord_x = intrinsic_matrix.at<double>(0, 0) * centroid_in_cam_coord3f.x +
                    intrinsic_matrix.at<double>(0, 2) * centroid_in_cam_coord3f.z,
          coord_y = intrinsic_matrix.at<double>(1, 1) * centroid_in_cam_coord3f.y +
                    intrinsic_matrix.at<double>(1, 2) * centroid_in_cam_coord3f.z;
   cv::Point centroid_in_cam_pixel_coord(coord_x/centroid_in_cam_coord3f.z, coord_y/centroid_in_cam_coord3f.z);
   /*double radius_square = pow(centroid(0), 2)+pow(centroid(1),2),
          undistort_x = centroid_in_cam_pixel_coord.x*(1+distortion_coeff.at<double>(0)*radius_square+
                                                      distortion_coeff.at<double>(1)*radius_square*radius_square),
          undistort_y = centroid_in_cam_pixel_coord.y*(1+distortion_coeff.at<double>(0)*radius_square+
                                                      distortion_coeff.at<double>(1)*radius_square*radius_square);
   centroid_in_cam_pixel_coord = cv::Point(undistort_x, undistort_y);*/
   if(inRange(centroid_in_cam_pixel_coord.x, 0, WIDTH) and inRange(centroid_in_cam_pixel_coord.y, 0, HEIGHT) and centroid_in_cam_coord3f.z >0){
     //ROS_INFO("%d\t%d", centroid_in_cam_pixel_coord.x, centroid_in_cam_pixel_coord.y);
     for(auto b = bbList.bounding_boxes.begin(); b != bbList.bounding_boxes.end(); ++b){
       if(inRange(centroid_in_cam_pixel_coord.x, b->xmin, b->xmax) and 
          inRange(centroid_in_cam_pixel_coord.y, b->ymin, b->ymax)){
         //ROS_INFO("Got one! With class: %s", b.Class.c_str());
         double prob_thres; if(!nh_.getParam("/darknet_ros/yolo_model/threshold/value", prob_thres)) prob_thres = 0.5f; 
         if(b->probability < prob_thres) continue; // Not reliable
         if(b->Class == "person") continue; // No person
         int index = (int)(b - bbList.bounding_boxes.begin());
         std::vector<int>::iterator it;
         it = find(indexList.begin(), indexList.end(), index);
         if(it==indexList.end()){ // Not found
           visualization_msgs::Marker marker;
           marker.id = id_count; ++id_count;
           marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
           marker.action = visualization_msgs::Marker::ADD;
           marker.header.frame_id = "velodyne";
           marker.scale.z = 1.0f;
           marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;
           marker.pose.position.x = centroid(0);
           marker.pose.position.y = centroid(1);
           marker.pose.position.z = centroid(2);
           marker.pose.orientation.w = 1.0f;
           marker.lifetime = ros::Duration(1.0f);
           marker.text = b->Class;
           markerArray.markers.push_back(marker);
           indexList.push_back(index); // Put index into indexList
         } else continue; // One bounding box should regist only one cluster
       }
     }
   } else continue; // Out of range, ignore
  } pub_markerArray.publish(markerArray);
}

void HW7::cb_bbList(const darknet_ros_msgs::BoundingBoxesConstPtr &bbListMsg){
  bbList =  *bbListMsg;
  //ROS_INFO("There are %d boxes", (int)bbList.bounding_boxes.size());
}

