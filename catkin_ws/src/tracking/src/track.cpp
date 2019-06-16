#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  // Initialize point cloud
  cloud_raw = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_filtered = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_current_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_last_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  // Subscriber and Publisher
  lidar_sub = nh_.subscribe("/points_raw", 1, &Track::cb_lidar, this);
  result_pub = pnh.advertise<visualization_msgs::MarkerArray>("text", 1);
  filtered_pub = pnh.advertise<sensor_msgs::PointCloud2>("filtered", 1);
  cluster_pub = pnh.advertise<sensor_msgs::PointCloud2>("cluster", 1);
  // ICP converge criteria
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  // Get parameters from launch file
  if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1f; ROS_INFO("leaf_size: %f", leaf_size);
  if(!pnh_.getParam("clusterDistThres", clusterDistThres)) clusterDistThres = 30.0f; ROS_INFO("clusterDistThres: %f", clusterDistThres);
  if(!pnh_.getParam("centroidDistThres", centroidDistThres)) centroidDistThres = 1.0f; ROS_INFO("centroidDistThres: %f", centroidDistThres);
  clusterTolerance = 0.5f;
  ROS_INFO("[%s] Node ready!", ros::this_node::getName().c_str());
}

void Track::process_data(void){
  while(TupleVector.size() <= 1 and ros::ok()){
    ROS_INFO("No enough data receive yet!");
    ros::Duration(0.3).sleep();
  }
  ROS_INFO("Start processing"); 
  ros::Time total_time = ros::Time::now();
  Matrix4f tf = Matrix4f::Identity(4, 4);
  while(TupleVector.size() > 1 and ros::ok()){
    ROS_INFO("process_data, size:%d", TupleVector.size());
    // Get the front two element in TupleVector
    DataTuple last = TupleVector.front();
    DataTuple current = TupleVector.at(1);
    // retrieve data in tuple
    Time time_last = get<0>(last);
    Time time_current = get<0>(current);
    CentroidVector centV_last = get<1>(last);
    CentroidVector centV_current = get<1>(current);
    PointCloudXYZIPtr cloud_last = get<3>(last);
    PointCloudXYZIPtr cloud_current = get<3>(current);
    // Find tf between two pointcloud
    PointCloudXYZI final_cloud;
    icp.setInputSource(cloud_last);
    icp.setInputTarget(cloud_current);
    icp.align(final_cloud, tf);
    tf = icp.getFinalTransformation();
    // Find centroid pair 
    int count = 0;
    ResultVector rv;
    for(auto cc : centV_current){
      for(auto cl : centV_last){
        double dist = sqrt(pow(cc(0)-cl(0), 2) + pow(cc(1)-cl(1), 2) + pow(cc(2)-cl(2), 2));
        if(dist < centroidDistThres and firstProcess){
          ResultTuple t = make_tuple(time_current, count, cc);
          rv.push_back(t);
          count ++;
        }
      }
    }
    RVector.push_back(rv);
    ROS_INFO("%d clusters are matched!", count);



    TupleVector.erase(TupleVector.begin()); 
  }
}

void Track::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg){
  ros::Time time_start = ros::Time::now();
  pcl::fromROSMsg(*lidarMsg, *cloud_raw);
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
  // Segment the largest planar component from the cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  // Extract inliers from input cloud
  PointCloudXYZIPtr cloud_inliers(new PointCloudXYZI);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_inliers);
  // Remove inliers
  PointCloudXYZIPtr cloud_remove_inliers(new PointCloudXYZI);
  extract.setNegative(true);
  extract.filter(*cloud_remove_inliers); 
  *cloud_filtered = *cloud_remove_inliers;
  // Calculate the value of mean and sd of intensity of cloud_filtered
  double mean = 0, x_square = 0;
  size_t points_num = cloud_filtered->size();
  for(auto point : *cloud_filtered){
    x_square += std::pow(point.intensity, 2);
    mean += point.intensity;
  }
  mean /= points_num;
  double sd = std::sqrt(x_square/points_num - std::pow(mean, 2));
  // Normalize intensity
  for(auto it=cloud_filtered->points.begin(); it!=cloud_filtered->points.end(); ++it){
    it->intensity = (it->intensity - mean) / sd;
  }
  // Pass Through filter, pretain cloud with intensity within [-3.89, 0]
  PointCloudXYZIPtr cloud_pt(new PointCloudXYZI);
  PassThrough<PointXYZI> pt;
  pt.setInputCloud(cloud_filtered);
  pt.setFilterFieldName("intensity");
  pt.setFilterLimits(-3.89, 0);
  pt.filter(*cloud_filtered);
  // Setup KD Tree
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);
  // Clear old vector
  cluster_indices.clear();
  // Euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance); // 
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(1500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  // Get cluster cloud
  int id_count = 0;
  ClusterVector clus_vec;
  CentroidVector cent_vec;
  PointCloudXYZIPtr cloud_clusters (new PointCloudXYZI);
  for(vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    PointCloudXYZIPtr cloud_cluster (new PointCloudXYZI);
    for(std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    }
    Vector4f centroid;
    compute3DCentroid(*cloud_cluster, centroid);
    double dist = sqrt(pow(centroid(0), 2) + pow(centroid(1), 2) + pow(centroid(2), 2));
    if(dist > clusterDistThres) continue; // Too far away, neglect
    clus_vec.push_back(*cloud_cluster);
    cent_vec.push_back(centroid);
    *cloud_clusters += *cloud_cluster; // add cluster cloud
  }
  DataTuple tuple = make_tuple(lidarMsg->header.stamp, cent_vec, clus_vec, cloud_filtered);
  TupleVector.push_back(tuple);

  Time time_end = Time::now();
  cout << "Process Time " << time_end-time_start << endl;
}