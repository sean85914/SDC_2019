#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  cloud_raw = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  cloud_inliers = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());

  //last_frame.header.stamp = 0;

  lidar_sub = nh_.subscribe("/points_raw", 1, &Track::cb_lidar, this);
  result_pub = pnh.advertise<visualization_msgs::MarkerArray>("text", 1);
  filtered_pub = pnh.advertise<sensor_msgs::PointCloud2>("filtered", 1);
  cluster_pub = pnh.advertise<sensor_msgs::PointCloud2>("cluster", 1);
  if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1f; ROS_INFO("leaf_size: %f", leaf_size);
  clusterTolerance = 0.5f;
  ROS_INFO("[%s] Node ready!", ros::this_node::getName().c_str());
}

void Track::pointCloudPreprocessing(void){
  //ROS_INFO("Start pointcloud preprocessing...");
  
  // Pass Through filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(cloud_raw);
  pt.setFilterFieldName("intensity");
  pt.setFilterLimits(0, 0.1);
  pt.filter(*cloud_pt);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_i(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_pt, *cloud_remove_i);
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_remove_i);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);

  vg.filter(*cloud_filtered);

  // Ground removal
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.25);
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Extract inliers from input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_inliers);

  // Remove inliers, extract the rest
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_inliers(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setNegative(true);
  extract.filter(*cloud_remove_inliers); 
  *cloud_filtered = *cloud_remove_inliers;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  // Clear old vector
  cluster_indices.clear();
  // Euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance); // 
  ec.setMinClusterSize(30);
  ec.setMaxClusterSize(2500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //ROS_INFO("Cluster size: %d", (int)cluster_indices.size());
}


void Track::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg){
  pcl::fromROSMsg(*lidarMsg, *cloud_raw);
  // clear marker array
  markerArray.markers.clear();
  pointCloudPreprocessing();
  int id_count = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    }
    *cloud_clusters += *cloud_cluster;
    vector_of_centroid voc;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    double dist = std::sqrt(centroid(0)*centroid(0) + centroid(1)*centroid(1) + centroid(2)*centroid(2));
    //if(dist > 50.0f) continue; // Too far away, neglect
    //ROS_INFO("%d\t%d", centroid_in_cam_pixel_coord.x, centroid_in_cam_pixel_coord.y);
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
    marker.text = std::to_string(marker.id);
    markerArray.markers.push_back(marker);
  }
  sensor_msgs::PointCloud2 cluster_cloud;
  pcl::toROSMsg(*cloud_clusters, cluster_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  cluster_pub.publish(cluster_cloud);
  result_pub.publish(markerArray);



}