#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  cloud_raw = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_inliers = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());

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
  // Remove tail of the car itself
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  pcl::PointXYZI origin; origin.x = 0.0f; origin.y = 0.0f; origin.z = 0.0f;
  std::vector<int> pointIdxRadiusSearch; // Vector to save inlier indices
  std::vector<float> pointRadiusSquaredDistance; // Vector to save corresponding distance
  kdtree.setInputCloud(cloud_raw);
  kdtree.radiusSearch(origin, MIN_DIS, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  extract.setInputCloud(cloud_raw);
  boost::shared_ptr<std::vector<int>> indicesPtr (new std::vector<int> (pointIdxRadiusSearch));
  extract.setIndices(indicesPtr);
  extract.setNegative(true); // Inliers will be erase
  extract.filter(*cloud_raw);
  // Calculate the mean and sd of point_raw's intensity
  double mean = 0;
  double x_square = 0;
  size_t points_num = cloud_raw->size();
  for(auto point : *cloud_raw){
    x_square += std::pow(point.intensity, 2);
    mean += point.intensity;
  }
  mean = mean / points_num;
  double sd = std::sqrt(x_square/points_num - std::pow(mean, 2));

  for(auto it=cloud_raw->points.begin(); it!=cloud_raw->points.end(); ++it){
    it->intensity = (it->intensity - mean) / sd;
  }

  // Pass Through filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(cloud_raw);
  pt.setFilterFieldName("intensity");
  pt.setFilterLimits(-3.89, 0);
  pt.filter(*cloud_pt);

  sensor_msgs::PointCloud2 cluster_cloud;
  pcl::toROSMsg(*cloud_pt, cluster_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  filtered_pub.publish(cluster_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remove_i(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud_pt, *cloud_remove_i);
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_remove_i);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);

  vg.filter(*cloud_filtered);

  // Ground removal
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.3);
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Extract inliers from input cloud
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_inliers);

  // Remove inliers, extract the rest
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remove_inliers(new pcl::PointCloud<pcl::PointXYZI>);
  extract.setNegative(true);
  extract.filter(*cloud_remove_inliers); 
  *cloud_filtered = *cloud_remove_inliers;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);
  // Clear old vector
  cluster_indices.clear();
  // Euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance); // 
  ec.setMinClusterSize(30);
  ec.setMaxClusterSize(1500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //ROS_INFO("Cluster size: %d", (int)cluster_indices.size());
}


void Track::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg){
  pcl::fromROSMsg(*lidarMsg, *cloud_raw);
  // clear marker array
  // FIXME: the issue of appearance of old data can solved this way, but the marker will flash which is not comfortable
  for(auto& marker: markerArray.markers) marker.action = visualization_msgs::Marker::DELETE; result_pub.publish(markerArray);
  markerArray.markers.clear();
  pointCloudPreprocessing();
  int id_count = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    double mean = 0;
    for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    }

    for(auto point : *cloud_cluster){
      mean += point.intensity;
    } 

    mean /= cloud_cluster->size();
    
    vector_of_centroid voc;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    double dist = std::sqrt(centroid(0)*centroid(0) + centroid(1)*centroid(1) + centroid(2)*centroid(2));
    if(dist > MAX_DIS) continue; // Too far away, neglect
    if(centroid[2] > MAX_HEIGHT) continue; // Centroid too high, neglect
    *cloud_clusters += *cloud_cluster; // XXX: neglected should not add into cloud_clusters? 
    mean /= cloud_cluster->size();
    *cloud_clusters += *cloud_cluster; // add cluster cloud
    voc.push_back(centroid); // add centroid for comparison

    // Visualize this cluster in Rviz
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
    //marker.lifetime = ros::Duration(1.0f);
    marker.text = std::to_string(centroid[2]);
    markerArray.markers.push_back(marker);
  }
  sensor_msgs::PointCloud2 cluster_cloud;
  pcl::toROSMsg(*cloud_clusters, cluster_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  cluster_pub.publish(cluster_cloud);
  result_pub.publish(markerArray);
}
