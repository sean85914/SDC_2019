#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  // Initialize point cloud
  cloud_raw = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_filtered = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_current_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_last_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  // Subscriber and Publisher
  lidar_sub = nh_.subscribe("/points_raw", 1, &Track::cb_lidar, this);
  result_pub = pnh_.advertise<visualization_msgs::MarkerArray>("text", 1);
  filtered_pub = pnh_.advertise<sensor_msgs::PointCloud2>("filtered", 1);
  cluster_pub = pnh_.advertise<sensor_msgs::PointCloud2>("cluster", 1);
  bb_pub = pnh_.advertise<visualization_msgs::MarkerArray>("box", 1);
  // ICP converge criteria
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-3);
  icp.setEuclideanFitnessEpsilon(1e-3);
  // Get parameters from launch file

  if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1f; ROS_INFO("leaf_size: %f", leaf_size);
  if(!pnh_.getParam("clusterDistThres", clusterDistThres)) clusterDistThres = 30.0f; ROS_INFO("clusterDistThres: %f", clusterDistThres);
  if(!pnh_.getParam("centroidDistThres", centroidDistThres)) centroidDistThres = 1.0f; ROS_INFO("centroidDistThres: %f", centroidDistThres);
  clusterTolerance = 0.5f;
  ROS_INFO("[%s] Node ready!", ros::this_node::getName().c_str());
}

void Track::check_bag_has_end(void){
  while(!startReceive){}
  while((ros::Time::now() - startReceiveTime).toSec() < 1){}
  bagHasEnd = true;
  ROS_INFO("Bag Has Reached End!");
}

void Track::process_data(void){
  while(TupleVector.size() == 0 and ros::ok()){
    ROS_INFO("No enough data receive yet!");
    ros::Duration(0.3).sleep();

  }
  ROS_INFO("Start processing"); 
  ros::Time total_time = ros::Time::now();
  Matrix4f tf = Matrix4f::Identity(4, 4);
  while(ros::ok()){
    DataTuple tuple_source = TupleVector.front();
    Time time_source = get<0>(tuple_source);
    CentroidVector centV_source = get<1>(tuple_source);
    ClusterVector clusV_source = get<2>(tuple_source);
    // First Process
    if(firstProcess){
      ResultVector rv_first;
      firstProcess = false;
      for(int count=0; count<centV_source.size(); ++count){
        ResultTuple rt = make_tuple(time_source, count, centV_source.at(count), clusV_source.at(count));
        rv_first.push_back(rt);
      }
      RVector.push_back(rv_first);
    }
    // Rest Process
    else if(TupleVector.size() > 1){
      ros::Time time_start = ros::Time::now();
      ROS_INFO("process_data, size:%d", (int)TupleVector.size());
      DataTuple tuple_target = TupleVector.at(1);
      Time time_target = get<0>(tuple_target);
      CentroidVector centV_target = get<1>(tuple_target);
      ClusterVector clusV_target = get<2>(tuple_target);
      PointCloudXYZIPtr cloud_source = get<3>(tuple_source);
      PointCloudXYZIPtr cloud_target = get<3>(tuple_target);
      // Find tf between two pointcloud
      ros::Time icp_t = ros::Time::now();
      PointCloudXYZI final_cloud;
      icp.setInputSource(cloud_source);
      icp.setInputTarget(cloud_target);
      icp.align(final_cloud, tf);
      tf = icp.getFinalTransformation();
      cout << "ICP spent time: " << ros::Time::now() - icp_t << endl;
      // Find centroid pair 
      ResultVector rv;
      int no_match_count = 0;
      ros::Time t = ros::Time::now();
      for(int target_count=0; target_count < centV_target.size(); ++target_count){
        ResultVector v_last = RVector.back();
        int idMax = get<1>(v_last.at(v_last.size() - 1));
        bool match = false;
        Vector4f ct = centV_target.at(target_count);
        for(int source_count=0; source_count < centV_source.size(); ++source_count){
          Vector4f cs = centV_source.at(source_count);
          double dist = sqrt(pow(ct(0)-cs(0), 2) + pow(ct(1)-cs(1), 2) + pow(ct(2)-cs(2), 2));
          if(dist < centroidDistThres){
            match = true;
            int id = get<1>(v_last.at(source_count));
            ResultTuple t = make_tuple(time_target, id, ct, clusV_target.at(target_count));
            rv.push_back(t);
          }
        }
        if(!match){// Cluster that isn't existed at last frame
          int id = idMax + no_match_count + 1;
          ResultTuple t = make_tuple(time_target, id, ct, clusV_target.at(target_count));
          rv.push_back(t);
          no_match_count++;
        }
      }
      RVector.push_back(rv);
      ROS_INFO("%d clusters are matched!, %d cluster aren't matched", (int)rv.size() - no_match_count, no_match_count);
      // Remove the first element
      TupleVector.erase(TupleVector.begin());
    }
    // Process the last element
    else if(bagHasEnd){
      ROS_INFO("All Data Are Processed");
      break;
    } 
  }
}

void Track::cb_lidar(const sensor_msgs::PointCloud2ConstPtr &lidarMsg){
  if(!startReceive){
    startReceiveTime = Time::now();
    startReceive = true;
  }
  startReceiveTime = Time::now();
  Time time_start = Time::now();
  fromROSMsg(*lidarMsg, *cloud_raw);
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_raw);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  // Ground removal
  SACSegmentation<PointXYZI> seg;
  PointIndices::Ptr inliers (new PointIndices);
  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.25);
  // Segment the largest planar component from the cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  // Extract inliers from input cloud
  PointCloudXYZIPtr cloud_inliers(new PointCloudXYZI);
  ExtractIndices<PointXYZI> extract;
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
  pcl::search::KdTree<PointXYZI>::Ptr tree (new pcl::search::KdTree<PointXYZI>);
  tree->setInputCloud(cloud_filtered);
  // Clear old vector
  cluster_indices.clear();
  // Euclidean cluster
  EuclideanClusterExtraction<PointXYZI> ec;
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
  cout << "PointCloud Filtering Process Time " << time_end-time_start << endl;
}

void Track::drawBoundingBox(vector<PointCloud<PointXYZI>> pcVec){
  visualization_msgs::MarkerArray boxArray;
  for(size_t i=0; i<pcVec.size(); ++i){
    PointXYZI minPoint, maxPoint;
    getMinMax3D(pcVec[i], minPoint, maxPoint);
    visualization_msgs::Marker bb;
    bb.header.frame_id = "velodyne";
    bb.ns = "box_" + std::to_string((int)i);
    bb.id = (int)i;
    bb.type = visualization_msgs::Marker::LINE_STRIP;
    bb.action = visualization_msgs::Marker::ADD;
    bb.pose.orientation.w = 1.0f;
    bb.scale.x = 0.1f;
    bb.color.r = bb.color.g = bb.color.b = bb.color.a = 1.0f;
    geometry_msgs::Point A, B, C, D, E, F, G, H;
    A.x = minPoint.x, A.y = minPoint.y, A.z = minPoint.z; 
    B.x = maxPoint.x, B.y = minPoint.y, B.z = minPoint.z;
    C.x = minPoint.x, C.y = maxPoint.y, C.z = minPoint.z;
    D.x = minPoint.x, D.y = minPoint.y, D.z = maxPoint.z;
    E.x = maxPoint.x, E.y = maxPoint.y, E.z = minPoint.z; 
    F.x = maxPoint.x, F.y = minPoint.y, F.z = maxPoint.z;
    G.x = minPoint.x, G.y = maxPoint.y, G.z = maxPoint.z;
    H.x = maxPoint.x, H.y = maxPoint.y, H.z = maxPoint.z;
    bb.points.push_back(A); bb.points.push_back(B); bb.points.push_back(E); bb.points.push_back(C); 
    bb.points.push_back(A); bb.points.push_back(D); bb.points.push_back(F); bb.points.push_back(H); 
    bb.points.push_back(G); bb.points.push_back(D); bb.points.push_back(F); bb.points.push_back(B); 
    bb.points.push_back(E); bb.points.push_back(H); bb.points.push_back(G); bb.points.push_back(C); 
    boxArray.markers.push_back(bb);
  }
  ROS_INFO("Cluster Size: %d", (int)pcVec.size());
  ROS_INFO("Marker Array Size: %d", (int)boxArray.markers.size());
  ROS_INFO("Marker Point Size: %d", (int)boxArray.markers[0].points.size());
  bb_pub.publish(boxArray);
}
