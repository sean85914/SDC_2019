#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  // Initialize point cloud
  cloud_raw = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_current_frame = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  cloud_last_frame = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI> ());
  // Subscriber and Publisher
  lidar_sub = nh_.subscribe("/points_raw", 1, &Track::cb_lidar, this);
  result_pub = pnh.advertise<visualization_msgs::MarkerArray>("text", 1);
  filtered_pub = pnh.advertise<sensor_msgs::PointCloud2>("filtered", 1);
  cluster_pub = pnh.advertise<sensor_msgs::PointCloud2>("cluster", 1);

  // ICP converge criteria
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);

  //rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("velodyne", "/text"));

  if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1f; ROS_INFO("leaf_size: %f", leaf_size);
  clusterTolerance = 0.5f;
  ROS_INFO("[%s] Node ready!", ros::this_node::getName().c_str());
}

void Track::pointCloudPreprocessing(void){
  //ROS_INFO("Start pointcloud preprocessing...");
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_raw);
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
  seg.setDistanceThreshold(0.25);
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Extract inliers from input cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_inliers);

  // Remove inliers, extract the rest
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_remove_inliers(new pcl::PointCloud<pcl::PointXYZI>);
  extract.setNegative(true);
  extract.filter(*cloud_remove_inliers); 
  *cloud_filtered = *cloud_remove_inliers;

  // Calculate the mean and sd of point_raw's intensity
  double mean = 0;
  double x_square = 0;
  size_t points_num = cloud_filtered->size();
  for(auto point : *cloud_filtered){
    x_square += std::pow(point.intensity, 2);
    mean += point.intensity;
  }
  mean = mean / points_num;
  double sd = std::sqrt(x_square/points_num - std::pow(mean, 2));

  for(auto it=cloud_filtered->points.begin(); it!=cloud_filtered->points.end(); ++it){
    it->intensity = (it->intensity - mean) / sd;
  }

  // Pass Through filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(cloud_filtered);
  pt.setFilterFieldName("intensity");
  pt.setFilterLimits(-3.89, -0);
  pt.filter(*cloud_filtered);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);
  // Clear old vector
  cluster_indices.clear();
  // Euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance); // 
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(1500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //ROS_INFO("Cluster size: %d", (int)cluster_indices.size());
}

void Track::comparison(void){
  if(cloud_last_frame->size() == 0)
    return;
  int objects_num = 0;
  Eigen::Matrix4f tf;
  icp.setInputSource(cloud_last_frame);
  icp.setInputTarget(cloud_current_frame);
  
}

void Track::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg){
  pcl::fromROSMsg(*lidarMsg, *cloud_raw);
  // clear marker array
  markerArray.markers.clear();
  //rviz_tools_->deleteAllMarkers();
  pointCloudPreprocessing();
  int id_count = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    double mean = 0; // mean intensity of current cluster
    for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      mean += cloud_filtered->points[*pit].intensity;
    }
    vector_of_centroid voc;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    double dist = std::sqrt(centroid(0)*centroid(0) + centroid(1)*centroid(1) + centroid(2)*centroid(2));
    if(dist > 30.0f) continue; // Too far away, neglect
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
    marker.lifetime = ros::Duration(1.0f);
    marker.text = std::to_string(mean);
    markerArray.markers.push_back(marker);
  }

  // 
  *cloud_current_frame = *cloud_clusters;
  comparison();

  sensor_msgs::PointCloud2 cluster_cloud;
  pcl::toROSMsg(*cloud_clusters, cluster_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  cluster_pub.publish(cluster_cloud);
  result_pub.publish(markerArray);
}