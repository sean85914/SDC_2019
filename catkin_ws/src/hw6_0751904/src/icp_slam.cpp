#include <icp_slam.h>

ICPSLAM::ICPSLAM(ros::NodeHandle nh, ros::NodeHandle pnh, bool offline): 
    nh_(nh), pnh_(pnh), offline_(offline), count(1) , status(0), mapDownsamplingCounter(1), dx(0.0), dy(0.0){
    package_path = ros::package::getPath("hw6_0751904");
    // Get parameters
    if(!pnh_.getParam("removeGround", removeGround)) removeGround = false; ROS_INFO("removeGround: %s", (removeGround==true?"true":"false"));
    if(!pnh_.getParam("use_icp", use_icp)) use_icp = true; ROS_INFO("use_icp: %s", (use_icp==true?"true":"false"));
    if(use_icp){
      if(!pnh_.getParam("point2point", point2point)) point2point = true; ROS_INFO("%s", point2point==1?"Use point to point ICP":"Use point to plane ICP");
    }
    if(!pnh_.getParam("numofscan", numofscan)){
      if(offline){
        ROS_ERROR("No numofscan provided, aborting...");
        status = -1; return;
      }
    } ROS_INFO("numofscan: %d", numofscan); 
    if(!pnh_.getParam("have_odom", have_odom)) have_odom = false; ROS_INFO("have_odom: %s", (have_odom==true?"true":"false"));
    if(!pnh_.getParam("saveEvery", saveEvery)) saveEvery = 10; ROS_INFO("saveEvery: %d", saveEvery);
    if(!pnh_.getParam("leaf_size", leaf_size)) leaf_size = 0.1; ROS_INFO("leaf_size: %f", leaf_size);
    if(!pnh_.getParam("plane_thin", plane_thin)) plane_thin = 0.3; ROS_INFO("plane_thin: %f", plane_thin);
    if(!pnh_.getParam("lower_z", lower_z)) lower_z = -5.0; ROS_INFO("lower_z: %f", lower_z);
    if(!pnh_.getParam("upper_z", upper_z)) upper_z = 5.0; ROS_INFO("upper_z: %f", upper_z);
    if(!pnh_.getParam("lower_r", lower_r)) lower_r = 1.0; ROS_INFO("lower_r: %f", lower_r);
    if(!pnh_.getParam("upper_r", upper_r)) upper_r = 10.0; ROS_INFO("upper_r: %f", upper_r);
    if(!pnh_.getParam("prefix", prefix)) prefix=""; ROS_INFO("prefix: %s", prefix.c_str());
    if(!pnh_.getParam("folder", folder)) folder="pcd"; ROS_INFO("folder: %s", folder.c_str());
    if(offline_) assert(numofscan!=0); // if offline, check numofscan is not zero
    if(removeGround) assert(plane_thin>0); // if removeGround, check plane_thin nonnegative
    assert(leaf_size>0); // leaf size should be nonnegative
    assert(upper_z>lower_z); // Make sure upper_z > lower_z
    assert(lower_r>=0 and upper_r>=0); // Make sure nonnegative radius
    assert(upper_r>lower_r); // Maker sure upper_r > lower_r
    assert(saveEvery>0);
    if(!offline_){
      if(!pnh_.getParam("len_of_vec", len_of_vec)) len_of_vec = 20; ROS_INFO("len_of_vec: %d", len_of_vec); assert(len_of_vec>0);
      sub_pc = pnh_.subscribe("velodyne_points", 1, &ICPSLAM::cb_pointcloud, this);
      // FIXME: if online then has to subscribe to odometry topic
    }
    if(have_odom and offline_){
      fin.open("odometry.txt"); if(!fin) {ROS_ERROR("Cannot open file, aborting..."); status = -1; return;}
      std::string line;
      std::getline(fin, line);
      std::stringstream ss(line);
      double _; // redundant variable placeholder
      ss >> origin.x >> origin.y >> _ >> _ >> _ >> origin.yaw;
      ROS_INFO("Read first odometry data: %f, %f, %f", origin.x, origin.y, origin.yaw);
      odomTransform = Eigen::Matrix3f::Zero();
      Eigen::Matrix2f rot;
      rot(0, 0) =  cos(origin.yaw); rot(0, 1) = sin(origin.yaw);
      rot(1, 0) = -sin(origin.yaw); rot(1, 1) = cos(origin.yaw);
      Eigen::Vector2f trans(-origin.x, -origin.y);
      odomTransform.block<2, 2>(0, 0) = rot;
      odomTransform.block<2, 1>(0, 2) = rot*trans;
      odomTransform(2, 2) = 1.0;
      std::cout << odomTransform << "\n";
    }
    pub_map = pnh_.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
    pub_path = pnh_.advertise<nav_msgs::Path>("path", 1);
    // Inital path
    path.header.frame_id = "map";
    // Set initial guess to identity
    guess = Eigen::Matrix4f::Identity(); 
    last_transform = guess;
    //past = guess;
    // ICP converge criteria
    if(use_icp){
      if(point2point){
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-10);
        icp.setEuclideanFitnessEpsilon(1e-8);
      } else{
        icp_plane.setMaxCorrespondenceDistance(1.0);
        icp_plane.setMaximumIterations(500);
        icp_plane.setTransformationEpsilon(1e-10);
        icp_plane.setEuclideanFitnessEpsilon(1e-8);
      }
    }else{
      ndt.setMaximumIterations(300);
      ndt.setTransformationEpsilon(1e-8);
      ndt.setEuclideanFitnessEpsilon(1e-6);
      ndt.setStepSize(0.1); // XXX: what this parameter means?
      ndt.setResolution(1.0); // XXX: what this parameter means?
    }
    if(offline){
      if(!start_process()) {status = -1; return;}
      std::string save_path = package_path + "/map.pcd";
      if(map.points.size() != 0){
        postprocessing(removeGround);
        publishMap();
        pcl::io::savePCDFileASCII(save_path, map);
        ROS_INFO("File: %s saved, there are %d points, existing...", save_path.c_str(), (int)map.points.size());
      }
      status = 1; ROS_INFO("Complete, shutdown...");
    }
    else ROS_INFO("----------------- Setup ready, waiting for message -----------------");
  }

void ICPSLAM::publishMap(void){
  // Publish pointcloud
  sensor_msgs::PointCloud2 pcout;
  /*pcl::PCLPointCloud2 pc2;
  pcl::toPCLPointCloud2(map, pc2); // First convert to pclpc2
  pcl_conversions::fromPCL(pc2, pcout);*/
  pcl::toROSMsg(map, pcout);
  pcout.header.frame_id = "map"; // Add frame id
  pub_map.publish(pcout); // Then publish
}

bool ICPSLAM::start_process(void){
  while(count<=numofscan && ros::ok()){
    ROS_INFO("----------------- %d -----------------", count);
    file_path = package_path + "/" + folder + "/" + (prefix==""?"":prefix+"_") + std::to_string(count) + ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, input) == -1){
      ROS_ERROR("Cannot find index %d, aborting...", count); return false;
    }
    if(count==1) { // First data
      map += input; ++count; 
      publishMap();
      geometry_msgs::PoseStamped p;
      path.poses.push_back(p);
      continue;
    }
    //past = guess;
    // Get odometry data and set as initial guess for ICP
    if(have_odom){
      std::string line;
      std::getline(fin, line);
      std::stringstream ss(line);
      OdomData odom;
      double _; // redundant
      ss >> odom.x >> odom.y >> _ >> _ >> _ >> odom.yaw;
      Eigen::Vector3f vec(odom.x, odom.y, 1.0);
      Eigen::Vector3f vec_transform = odomTransform*vec;
      std::cout << vec_transform << "\n";
      for(int i=0; i<2; ++i) guess(i, 3) = vec_transform(i);
    }
    //std::cout << guess << "\n";
    // Do registration
    PointXYZ filtered_pc = preprocessing(input.makeShared());
    ROS_INFO("Scan %d | %d -> %d", count, (int)input.points.size(), (int)filtered_pc.points.size());
    PointXYZ transformed_pc;
    ros::Time now = ros::Time::now();
    if(use_icp){
      icp.setInputSource(filtered_pc.makeShared());
      icp.setInputTarget(map.makeShared());
      icp.align(transformed_pc, guess);
      ROS_INFO("ICP time: %f", (ros::Time::now() - now).toSec());
      // Update initial guess for nect input
      guess = icp.getFinalTransformation(); 
    } else{
      ndt.setInputSource(filtered_pc.makeShared());
      ndt.setInputTarget(map.makeShared());
      ndt.align(transformed_pc, guess);
      // Update initial guess for nect input
      guess = ndt.getFinalTransformation(); 
    }
    geometry_msgs::PoseStamped p;
    p.pose.position.x = guess(0, 3);
    p.pose.position.y = guess(1, 3);
    p.pose.position.z = guess(2, 3);
    path.poses.push_back(p);
    ROS_INFO("(%f, %f, %f)", guess(0, 3), guess(1, 3), guess(2, 3));
    pub_path.publish(path);
    // Update map if needed
    if(count%saveEvery==0){ 
      pcl::transformPointCloud(input_filtered, transformed_pc, guess);
      if(icp.hasConverged()){
        ROS_INFO("Has converged! Add to map...");
        map += transformed_pc;
        ROS_INFO("%d points add to map, now there are %d points in map", (int)transformed_pc.points.size(), (int)map.points.size());
        //++mapDownsamplingCounter; if(mapDownsamplingCounter%saveEvery==0) mapDownsampling();
      }
      // Publish 
      publishMap();
    } ++count;
    //for(int i=0; i<3; ++i) guess(i, 3) += guess(i, 3) - past(i, 3);
  } // End while
  if(count==numofscan+1) return true;
  else return false;
}

void ICPSLAM::mapDownsampling(void){
  ROS_INFO("Map downsampling...");
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(map.makeShared());
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(map); ROS_INFO("Now map has %d points.", (int)map.points.size());
}

PointXYZ ICPSLAM::preprocessing(PointXYZPtr pc){
  PointXYZ out;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pc);
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(out);
  // Distance filter using KD tree
  //   1. Distance larger than upper_r will be neglected
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ origin(0.0, 0.0, 0.0);
  std::vector<int> pointIdxRadiusSearch; // Vector to save inlier indices
  std::vector<float> pointRadiusSquaredDistance; // Vector to save corresponding distance
  if(upper_r>0){ // Equal 0 -> doesn;t have to do this
    kdtree.setInputCloud(out.makeShared());
    kdtree.radiusSearch(origin, upper_r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    extract.setInputCloud(out.makeShared());
    // Convert std::vector<int> to pcl::PointIndices
    boost::shared_ptr<std::vector<int>> indicesPtr (new std::vector<int> (pointIdxRadiusSearch));
    extract.setIndices(indicesPtr);
    extract.setNegative(false);
    extract.filter(out);
  }
  //   2. Distance smaller than lower_r will be neglected
  if(lower_r>0){ // Equal 0 -> doesn;t have to do this
    kdtree.setInputCloud(out.makeShared());
    // Clear container vector
    pointIdxRadiusSearch.clear(); pointRadiusSquaredDistance.clear();
    kdtree.radiusSearch(origin, lower_r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    extract.setInputCloud(out.makeShared());
    boost::shared_ptr<std::vector<int>> indicesPtr (new std::vector<int> (pointIdxRadiusSearch));
    extract.setIndices(indicesPtr);
    extract.setNegative(true); // Inliers will be erase
    extract.filter(out);
  }
  // Copy out to input_filter, which contains only point in range [lower_r, upper_r]
  pcl::copyPointCloud(out, input_filtered);
  // Pass through filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(out.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (lower_z, upper_z);
  pass.filter(out);
  return out;
}

void ICPSLAM::postprocessing(bool removeGround){
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(map.makeShared());
  vg.setLeafSize (leaf_size, leaf_size, leaf_size);
  vg.filter(map);
  if(removeGround){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
    // Optional
    seg_plane.setOptimizeCoefficients(true);
    // Mandatory
    seg_plane.setModelType (pcl::SACMODEL_PLANE);
    seg_plane.setMethodType (pcl::SAC_RANSAC);
    seg_plane.setDistanceThreshold(plane_thin);
    seg_plane.setInputCloud (map.makeShared());
    seg_plane.segment (*inliers, *coefficients);
    ROS_INFO("There are %d ground inliers\n \
              Ground coefficeints: [%f] x + [%f] y + [%f] z = [%f]",
              (int)inliers->indices.size(),
              coefficients->values[0], coefficients->values[1], 
              coefficients->values[2], coefficients->values[3]*-1);
    pcl::ExtractIndices<pcl::PointXYZ> indiceFilter(true); // Initializing with true will allow us to extract the removed indices
    indiceFilter.setInputCloud(map.makeShared());
    indiceFilter.setIndices(inliers);
    indiceFilter.setNegative(true);
    indiceFilter.filter(map);
  }
}

void ICPSLAM::cb_pointcloud(const sensor_msgs::PointCloud2ConstPtr msg){
  pcl::fromROSMsg(*msg, input);
  PointXYZ filtered_pc = preprocessing(input.makeShared());
  if(count==1){
    map += input_filtered; ++count;
    publishMap(); 
    geometry_msgs::PoseStamped p;
    path.poses.push_back(p);
    last = ros::Time::now();
    pc_v.push_back(input_filtered); return;
  }
  PointXYZ batch_pc, transformed_pc;
  PointNormal transformed_pc_normal;
  for(size_t i=0; i<pc_v.size(); ++i) batch_pc += pc_v[i];
  if(use_icp){
    if(point2point){
      icp.setInputSource(filtered_pc.makeShared());
      icp.setInputTarget(batch_pc.makeShared());
      icp.align(transformed_pc, guess);
      // Update initial guess for nect input
      guess = icp.getFinalTransformation();
    }
    else{
      // Compute normal
      ros::Time icp_now = ros::Time::now();
      Normal batch_normal, input_normal;
      PointNormal batch_with_normal, input_with_normal;
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(batch_pc.makeShared());
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(leaf_size*2);
      ne.compute(batch_normal);
      pcl::concatenateFields(batch_pc, batch_normal, batch_with_normal);
      // Redundant
      std::vector<int> _;
      // Remove points with nan normals
      pcl::removeNaNNormalsFromPointCloud(batch_with_normal, batch_with_normal, _);
      ne.setInputCloud(filtered_pc.makeShared());
      ne.compute(input_normal);
      pcl::concatenateFields(filtered_pc, input_normal, input_with_normal);
      pcl::removeNaNNormalsFromPointCloud(input_with_normal, input_with_normal, _);
      guess(0, 3) += dx * (ros::Time::now() - last).toSec();
      guess(1, 3) += dy * (ros::Time::now() - last).toSec();
      // ICP
      icp_plane.setInputSource(input_with_normal.makeShared());
      icp_plane.setInputTarget(batch_with_normal.makeShared());
      icp_plane.align(transformed_pc_normal, guess);
      last_transform = guess;
      guess = icp_plane.getFinalTransformation();
      pcl::copyPointCloud(transformed_pc_normal, transformed_pc);
      ROS_INFO("ICP time: %f", (ros::Time::now()-icp_now).toSec());
    }
  } else{ // NDT
    ndt.setInputSource(filtered_pc.makeShared());
    ndt.setInputTarget(map.makeShared());
    ndt.align(transformed_pc, guess);
    // Update initial guess for nect input
    guess = ndt.getFinalTransformation(); 
  }
  geometry_msgs::PoseStamped p;
  p.pose.position.x = guess(0, 3);
  p.pose.position.y = guess(1, 3);
  p.pose.position.z = guess(2, 3);
  path.poses.push_back(p);
  double dt =  (ros::Time::now() - last).toSec();
  dx = (guess(0, 3) - last_transform(0, 3))/dt;
  dy = (guess(1, 3) - last_transform(1, 3))/dt;
  last = ros::Time::now();
  //ROS_INFO("(%f, %f, %f)", guess(0, 3), guess(1, 3), guess(2, 3));
  pub_path.publish(path);
  pcl::transformPointCloud(input_filtered, transformed_pc, guess);
  // Update map if needed
  if(count%saveEvery==0){ 
    if(point2point){
      if(icp.hasConverged()){
        ROS_INFO("Has converged! Add to map...");
        map += transformed_pc;
        ROS_INFO("%d points add to map, now there are %d points in map", (int)transformed_pc.points.size(), (int)map.points.size());
        ++mapDownsamplingCounter; if(mapDownsamplingCounter%saveEvery==0) mapDownsampling();
      }
    }
    else{
      if(icp_plane.hasConverged()){
        ROS_INFO("Has converged! Add to map...");
        map += transformed_pc;
        ROS_INFO("%d points add to map, now there are %d points in map", (int)transformed_pc.points.size(), (int)map.points.size());
        ++mapDownsamplingCounter; if(mapDownsamplingCounter%saveEvery==0) mapDownsampling();
      }
    }
    // Publish 
    publishMap();
  }
  if(count>=len_of_vec){
    pc_v.erase(pc_v.begin());
    pc_v.push_back(transformed_pc);
  } else pc_v.push_back(transformed_pc);
  ++count;
  return;
}

void ICPSLAM::saveMap(void){
  if(map.points.size() != 0){
    std::string save_path = package_path + "/map.pcd";
    pcl::io::savePCDFileASCII(save_path, map);
    ROS_INFO("File: %s saved, there are %d points, existing...", save_path.c_str(), (int)map.points.size());
  }
}

void ICPSLAM::mySigintHandler(int sig){
  ROS_WARN("Receive interrupt signal!");
  if(!offline_) {postprocessing(removeGround); saveMap(); publishMap(); ros::Duration(1.0).sleep();}
  pub_map.shutdown(); pub_path.shutdown(); if(!offline_) sub_pc.shutdown();
  if(have_odom) fin.close();
  ros::shutdown();
}
