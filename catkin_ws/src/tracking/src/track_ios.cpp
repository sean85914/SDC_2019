#include "track_ios.h"

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

  bicp_pub = pnh_.advertise<sensor_msgs::PointCloud2>("before_icp", 1);
  aicp_pub = pnh_.advertise<sensor_msgs::PointCloud2>("after_icp", 1);

  // ICP converge criteria
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-3);
  icp.setEuclideanFitnessEpsilon(1e-3);
  // Get parameters from launch file

  if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1f; ROS_INFO("leaf_size: %f", leaf_size);
  if(!pnh_.getParam("clusterDistThres", clusterDistThres)) clusterDistThres = 30.0f; ROS_INFO("clusterDistThres: %f", clusterDistThres);
  if(!pnh_.getParam("centroidDistThres", centroidDistThres)) centroidDistThres = 1.0f; ROS_INFO("centroidDistThres: %f", centroidDistThres);
  if(!pnh_.getParam("filename", filename)) filename="result"; ROS_INFO("filename: %s", filename.c_str());
  if(!pnh_.getParam("IoSThres", IoSThres)) IoSThres=0.5f; ROS_INFO("IoSThres: %f", IoSThres);
  clusterTolerance = 0.4f;
  writeFile.setFileName(package_path + "/" + filename + ".csv");
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
    if(TupleVector.size() == 0){
      ROS_INFO("Process too fast!!");
      continue;
    }
    DataTuple tuple_source = TupleVector.front();
    Time time_source = get<0>(tuple_source);
    CentroidVector centV_source = get<1>(tuple_source);
    ClusterVector clusV_source = get<2>(tuple_source);
    ResultVector rv;
    // First Process, save all data.
    if(firstProcess){
      firstProcess = false;
      for(int count=0; count<centV_source.size(); ++count){
        ResultTuple rt = make_tuple(time_source, count, centV_source.at(count), clusV_source.at(count));
        rv.push_back(rt);
      }
      RVector.push_back(rv);
      max_id_count = centV_source.size();
    }
    // Rest Process
    else if(TupleVector.size() > 1){
      //get last frame result
      ResultVector rv_last = RVector.back();
      //ros::Time time_start = ros::Time::now();
      ROS_INFO("process_data, size:%d", (int)TupleVector.size());
      DataTuple tuple_target = TupleVector.at(1);
      Time time_target = get<0>(tuple_target);
      CentroidVector centV_target = get<1>(tuple_target);
      ClusterVector clusV_target = get<2>(tuple_target);
      PointCloudXYZI cloud_source = get<3>(tuple_source);
      PointCloudXYZI cloud_target = get<3>(tuple_target);
      int match_count = 0, mis_count = 0, new_count = 0;
      // Find tf between two pointcloud
      ros::Time icp_t = ros::Time::now();
      PointCloudXYZI final_cloud;
      icp.setInputSource(cloud_source.makeShared());
      icp.setInputTarget(cloud_target.makeShared());
      icp.align(final_cloud, tf);
      tf = icp.getFinalTransformation();
      //cout << "ICP spent time: " << ros::Time::now() - icp_t << endl;
      // plot final_cloud and target cloud
      PointCloud<PointXYZRGB> b_cloud, a_cloud;
      copyPointCloud(cloud_target, b_cloud);
      copyPointCloud(final_cloud, a_cloud);
      for(auto &p : b_cloud.points) p.r = 255;
      for(auto &p : a_cloud.points) p.b = 255;
      sensor_msgs::PointCloud2 b_cloud_msg, a_cloud_msg;
      toROSMsg(a_cloud, a_cloud_msg);
      toROSMsg(b_cloud, b_cloud_msg);
      bicp_pub.publish(b_cloud_msg);
      aicp_pub.publish(a_cloud_msg);
      // match cluster by IoS
      for(int target_count=0; target_count < centV_target.size(); ++target_count){
        PointCloudXYZI cloud_t = clusV_target.at(target_count);
        double misIoS = 0; // IoS value for mis-classified cluster
        double matchIoS = 0; // max IoS value of two cluster
        double matchIoSInv = 0; // the inv IoS of the max IoS 
        bool match = false;
        ResultTuple match_tuple;
        for(int source_count=0; source_count < rv_last.size(); ++source_count){
          ResultTuple rtuple_source = rv_last.at(source_count);
          PointCloudXYZI cloud_s = get<3>(rtuple_source);
          double IoS = calculate_intersection_ratio(cloud_s, cloud_t, tf);
          //ROS_INFO("IoS: %f", IoS);
          double inv_IoS = calculate_intersection_ratio(cloud_t, cloud_s, tf.inverse());
          if(inv_IoS > misIoS) misIoS = inv_IoS;
          if(IoS > matchIoS){// matched, save id
            match = true;
            matchIoS = IoS;
            matchIoSInv = inv_IoS;
            match_tuple = rtuple_source;
          }
        }
        if(match){
          match_count++;
          ResultTuple t;
          //if(matchIoS > matchIoSInv) // bounding box in this frame is smaller than last, save cluster at last frame
            //t = make_tuple(time_target, get<1>(match_tuple), get<2>(match_tuple), get<3>(match_tuple));
          //else // bounding box in this frame is larger than last, save cluster at this frame
          PointCloudXYZI cluster = clusV_target.at(target_count);
          for(auto &p : cluster)
            p.intensity = 255*target_count/clusV_target.size();
          t = make_tuple(time_target, get<1>(match_tuple), centV_target.at(target_count), cluster);
          rv.push_back(t);
        }
        else{// no match, could be mis-cluster or new cluster
          if(misIoS > IoSThres){
            mis_count++;
            continue; //mis-cluster, ignore
          } 
          else{// new cluster
            new_count++;
            max_id_count++;
            ResultTuple t = make_tuple(time_target, max_id_count, centV_target.at(target_count), clusV_target.at(target_count));
            rv.push_back(t);
          }
        }
      }
      ROS_INFO("match: %d, mis: %d, new: %d", match_count, mis_count, new_count); 
      // Concatenate cluster with same id
      for(ResultVector::iterator it=rv.begin(); it != rv.end(); ++it){

      }
      RVector.push_back(rv);
      //ROS_INFO("%d clusters are matched!, %d cluster aren't matched", (int)rv.size() - no_match_count, no_match_count);
      // Remove the first element
      TupleVector.erase(TupleVector.begin());
    }
    // Process the last element
    else if(bagHasEnd){
      ROS_INFO("All Data Are Processed");
      break;
    }
    plotResult(rv); 
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
  extract.filter(*cloud_filtered); 
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
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(1000);
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
    if(dist > clusterDistThres or dist < 0.5 or centroid(2) > 0.1) continue; // Too far away, neglect
    clus_vec.push_back(*cloud_cluster);
    cent_vec.push_back(centroid);
    *cloud_clusters += *cloud_cluster; // add cluster cloud
  }
  DataTuple tuple = make_tuple(lidarMsg->header.stamp, cent_vec, clus_vec, *cloud_filtered);
  TupleVector.push_back(tuple);

  Time time_end = Time::now();
  //cout << "PointCloud Filtering Process Time " << time_end-time_start << endl;
}

double Track::calculate_intersection_ratio(const PointCloudXYZI cloud_source, const PointCloudXYZI cloud_target, const Matrix4f tf){
  PointXYZI minPoint_source, minPoint_target, maxPoint_source, maxPoint_target;
  getMinMax3D(cloud_source, minPoint_source, maxPoint_source);
  getMinMax3D(cloud_target, minPoint_target, maxPoint_target);
  Vector4f min_xyz = tf * Vector4f(minPoint_source.x, minPoint_source.y, minPoint_source.z, 0);
  Vector4f max_xyz = tf * Vector4f(maxPoint_source.x, maxPoint_source.y, maxPoint_source.z, 0);
  double x_min = max(min_xyz(0), minPoint_target.x);
  double x_max = min(max_xyz(0), maxPoint_target.x);
  double y_min = max(min_xyz(1), minPoint_target.y);
  double y_max = min(max_xyz(1), maxPoint_target.y);
  double z_min = max(min_xyz(2), minPoint_target.z);
  double z_max = min(max_xyz(2), maxPoint_target.z);
  if(x_max < x_min or y_max < y_min or z_max < z_min) return 0;
  //ROS_INFO("x_min: %f, x_max: %f, y_min: %f, y_max: %f, z_min: %f, z_max: %f",min_xyz(0), max_xyz(0), min_xyz(1), max_xyz(1), min_xyz(2), max_xyz(2));
  //ROS_INFO("tf_x_min: %f, tf_x_max: %f, tf_y_min: %f, tf_y_max: %f, tf_z_min: %f, tf_z_max: %f", minPoint_target.x, maxPoint_target.x, minPoint_target.y, maxPoint_target.y, minPoint_target.z, maxPoint_target.z);
  double intersectArea = (x_max - x_min) * (y_max - y_min) * (z_max - z_min);
  double targetArea = (maxPoint_target.x - minPoint_target.x) * (maxPoint_target.y - minPoint_target.y) * (maxPoint_target.z - minPoint_target.z);
  return intersectArea / targetArea;
}

void Track::plotResult(ResultVector rv){
  visualization_msgs::MarkerArray boxArray, idArray;
  PointCloudXYZI clusterCloud;
  for(size_t i=0; i<rv.size(); ++i){
    ResultTuple rt = rv.at(i);
    ros::Time stamp = get<0>(rt);
    int id = get<1>(rt);
    Vector4f centroid = get<2>(rt);
    PointCloudXYZI pc = get<3>(rt);
    clusterCloud += pc;
    PointXYZI minPoint, maxPoint;
    getMinMax3D(pc, minPoint, maxPoint);
    if((maxPoint.z - minPoint.z) > 1.5) continue; // Filter out trees
    if((maxPoint.z - minPoint.z) < 0.3) continue; // Filter out don't know what
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
    idMarker.scale.z = 1.2f;
    idMarker.color.g = idMarker.color.a = 1.0f;
    idMarker.color.r = idMarker.color.b = 0.0f;
    idMarker.lifetime = Duration(1.0f);
    idMarker.text = to_string(id);
    idArray.markers.push_back(idMarker);

    writeFile << stamp
              << id
              << (minPoint.x + maxPoint.x) / 2
              << (minPoint.y + maxPoint.y) / 2
              << (minPoint.z + maxPoint.z) / 2 << endrow;
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
