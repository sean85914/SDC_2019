#include "track.h"

Track::Track(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){

  // Initialize point cloud
  cloud_raw = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_filtered = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_current_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  cloud_last_frame = PointCloudXYZIPtr (new PointCloudXYZI ());
  // Subscriber and Publisher
  lidar_sub = nh_.subscribe("/points_raw", 1, &Track::cb_lidar, this);
  cluster_pub = pnh_.advertise<sensor_msgs::PointCloud2>("cluster", 1);
  bb_pub = pnh_.advertise<visualization_msgs::MarkerArray>("box", 1);
  id_pub = pnh_.advertise<visualization_msgs::MarkerArray>("id", 1);
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
    ResultVector rv;
    // First Process
    if(firstProcess){
      firstProcess = false;
      for(int count=0; count<centV_source.size(); ++count){
        ResultTuple rt = make_tuple(time_source, count, centV_source.at(count), clusV_source.at(count));
        rv.push_back(rt);
      }
      RVector.push_back(rv);
    }
    // Rest Process
    else if(TupleVector.size() > 1){
      ros::Time time_start = ros::Time::now();
      ROS_INFO("process_data, size:%d", (int)TupleVector.size());
      DataTuple tuple_target = TupleVector.at(1);
      Time time_target = get<0>(tuple_target);
      CentroidVector centV_target = get<1>(tuple_target);
      ClusterVector clusV_target = get<2>(tuple_target);
      PointCloudXYZI cloud_source = get<3>(tuple_source);
      PointCloudXYZI cloud_target = get<3>(tuple_target);
      // Find tf between two pointcloud
      ros::Time icp_t = ros::Time::now();
      PointCloudXYZI final_cloud;
      icp.setInputSource(cloud_source.makeShared());
      icp.setInputTarget(cloud_target.makeShared());
      icp.align(final_cloud, tf);
      tf = icp.getFinalTransformation();
      cout << "ICP spent time: " << ros::Time::now() - icp_t << endl;
      // Find centroid pair 
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
    drawBoundingBox(rv); 
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
    if(dist > clusterDistThres or dist < 2 or centroid(2) > 0.2) continue; // Too far away, neglect
    clus_vec.push_back(*cloud_cluster);
    cent_vec.push_back(centroid);
    *cloud_clusters += *cloud_cluster; // add cluster cloud
  }
  DataTuple tuple = make_tuple(lidarMsg->header.stamp, cent_vec, clus_vec, *cloud_filtered);
  TupleVector.push_back(tuple);

  Time time_end = Time::now();
  cout << "PointCloud Filtering Process Time " << time_end-time_start << endl;
}

void Track::drawBoundingBox(ResultVector rv){
  visualization_msgs::MarkerArray boxArray, idArray;
  PointCloudXYZI clusterCloud;
  for(size_t i=0; i<rv.size(); ++i){
    ResultTuple rt = rv.at(i);
    int id = get<1>(rt);
    Vector4f centroid = get<2>(rt);
    PointCloudXYZI pc = get<3>(rt);
    clusterCloud += pc;
    PointXYZI minPoint, maxPoint;
    getMinMax3D(pc, minPoint, maxPoint);
    visualization_msgs::Marker bbMarker, idMarker;
    bbMarker.header.frame_id = "velodyne";
    bbMarker.ns = "box_" + std::to_string((int)i);
    bbMarker.id = (int)i;
    bbMarker.type = visualization_msgs::Marker::LINE_STRIP;
    bbMarker.action = visualization_msgs::Marker::ADD;
    bbMarker.pose.orientation.w = 1.0f;
    bbMarker.scale.x = 0.1f;
    bbMarker.color.r = bbMarker.color.g = bbMarker.color.b = bbMarker.color.a = 1.0f;
    geometry_msgs::Point A, B, C, D, E, F, G, H;
    A.x = minPoint.x, A.y = minPoint.y, A.z = minPoint.z; 
    B.x = maxPoint.x, B.y = minPoint.y, B.z = minPoint.z;
    C.x = minPoint.x, C.y = maxPoint.y, C.z = minPoint.z;
    D.x = minPoint.x, D.y = minPoint.y, D.z = maxPoint.z;
    E.x = maxPoint.x, E.y = maxPoint.y, E.z = minPoint.z; 
    F.x = maxPoint.x, F.y = minPoint.y, F.z = maxPoint.z;
    G.x = minPoint.x, G.y = maxPoint.y, G.z = maxPoint.z;
    H.x = maxPoint.x, H.y = maxPoint.y, H.z = maxPoint.z;
    bbMarker.points.push_back(A); bbMarker.points.push_back(B); bbMarker.points.push_back(E); bbMarker.points.push_back(C); 
    bbMarker.points.push_back(A); bbMarker.points.push_back(D); bbMarker.points.push_back(F); bbMarker.points.push_back(H); 
    bbMarker.points.push_back(G); bbMarker.points.push_back(D); bbMarker.points.push_back(F); bbMarker.points.push_back(B); 
    bbMarker.points.push_back(E); bbMarker.points.push_back(H); bbMarker.points.push_back(G); bbMarker.points.push_back(C); 
    bbMarker.lifetime = Duration(0.5f);
    boxArray.markers.push_back(bbMarker);

    idMarker.header.frame_id = "velodyne";
    idMarker.ns = "id_" + std::to_string((int)i);
    idMarker.id = (int)i;
    idMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    idMarker.action = visualization_msgs::Marker::ADD;
    idMarker.pose.position.x = centroid(0);
    idMarker.pose.position.y = centroid(1);
    idMarker.pose.position.z = centroid(2);
    idMarker.pose.orientation.w = 1.0f;
    idMarker.scale.z = 1.0f;
    idMarker.color.r = idMarker.color.g = idMarker.color.b = idMarker.color.a = 1.0f;
    idMarker.lifetime = Duration(1.0f);
    idMarker.text = to_string(id);
    idArray.markers.push_back(idMarker);
  }
  //ROS_INFO("Cluster Size: %d", (int)pcVec.size());
  //ROS_INFO("Marker Array Size: %d", (int)boxArray.markers.size());
  //ROS_INFO("Marker Point Size: %d", (int)boxArray.markers[0].points.size());
  sensor_msgs::PointCloud2 pcMsg;
  toROSMsg(clusterCloud, pcMsg);
  pcMsg.header.frame_id = "velodyne";
  cluster_pub.publish(pcMsg);
  bb_pub.publish(boxArray);
  id_pub.publish(idArray);
}
