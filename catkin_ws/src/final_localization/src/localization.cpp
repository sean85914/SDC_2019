// C++ STL
#include <thread> // std::thread, C++11
#include <algorithm>    // std::min_element
// Boost
#include <boost/filesystem.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
// MSG
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// PCL
#include <pcl/io/pcd_io.h> // loadPCDFile
#include <pcl/point_types.h> // PointXYZ
#include <pcl/common/transforms.h> // Transform pointcloud
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
//#include <pcl/filters/approximate_voxel_grid.h> // Approximate VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/registration/ndt.h> // NDT
#include <pcl/filters/passthrough.h> // passThrough
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC segmentation
#include <pcl/filters/extract_indices.h> // Indice filter
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
// GeographicLib
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
// Self-defined
#include "csvfile.h" // csvfile

// Degree& Radian conversions
#define deg2rad(x) (x*M_PI/180.0)
#define rad2deg(x) (x*180.0/M_PI)


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;
typedef std::tuple<std::vector<double>, PointCloudXYZ, ros::Time> MyTuple;
typedef std::pair<Eigen::Matrix4f, ros::Time> transformationAndStamp;

/*
   ____   _          _      ____    ____  
  / ___| | |        / \    / ___|  / ___| 
 | |     | |       / _ \   \___ \  \___ \ 
 | |___  | |___   / ___ \   ___) |  ___) |
  \____| |_____| /_/   \_\ |____/  |____/ 
                                          
*/

class Localization{
 private:
  bool status; // Whether if finish, false if not and true if finish
  bool use_icp; // Whether using icp, using ndt if false
  bool save_pcd; // If save pcd for each scan, from ROS parameter server
  bool removeGround; // If remove ground of scan
  bool removeOutlier; // If remove outlier using statistical outlier removal
  bool passThrough; // If using passThrough filter to build local map pointcloud
  bool skip_mode; // If using skip mode, default is false, turn to true when low fitness error meet 
  int counter; // GPS frame counter
  int rotate_time; // First scan rotate times, from ROS parameter server
  int skip_num; // Skip number, only used when skip_mode turns on, from ROS parameter server
  int skip_counter;
  const int MAX_ITER = 1000; // Maximum iteration for pointcloud registration
  double last_x, last_y, last_z; // Last GPS reported position
  double rough_x, rough_y, rough_z; // Rough GPS reported position
  double radius; // Pointcloud search radius
  double length; // Length for passThrough to build local map
  double time_offset; // Time offset for rotating
  const double VG_SIZE = 0.25f; // Voxelgrid leaf size
  const double FREQ = 0.5f; // Frequency fot publishing map pointcloud, 2 seconds
  const double CONVERGE_SCORE = 0.2; // Score to consider as converged of registration (i.e., approch to ground truth)
  const double INIT_LAT = 24.7855644226f; // Initial latitude, from slide
  const double INIT_LON = 120.997009277f; // Initial longiture, from slide
  const double INIT_ALT = 127.651f; // Initial altitude, from slide
  std::string filename; // Write file name
  const std::string package_path = ros::package::getPath("final");
  const std::string FRAME = "map";
  const std::vector<std::string> pcd_name_vec = 
  {"first-0.pcd", "first-1.pcd", "first-2.pcd", "first-3.pcd", "first-4.pcd", "first-5.pcd",
   "first-6.pcd", "first-7.pcd", "first-8.pcd", "first-9.pcd", "first-10.pcd", "first-11.pcd",
   "first-12.pcd", "first-13.pcd", "first-14.pcd", "second-0.pcd", "second-1.pcd", "second-2.pcd",
   "submap_0.pcd", "submap_1.pcd", "submap_2.pcd", "submap_3.pcd", "submap_4.pcd", "submap_5.pcd",
   "submap_6.pcd", "submap_7.pcd", "submap_8.pcd", "submap_9.pcd", "submap_10.pcd", "submap_11.pcd",
   "submap_12.pcd", "submap_13.pcd", "submap_14.pcd", "submap_15.pcd","submap_16.pcd",
   "submap_17.pcd","submap_18.pcd","submap_19.pcd","submap_20.pcd"};
  GeographicLib::Geocentric earth;
  GeographicLib::LocalCartesian proj;
  Eigen::Matrix4f guess; // Initial guess placeholder
  csvfile writeFile;
  std::vector<MyTuple> myVector;
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pc;
  ros::Subscriber sub_gps;
  // Publishers
  ros::Publisher pub_map_pc;
  ros::Publisher pub_local_scan;
  ros::Publisher pub_raw;
  ros::Publisher pub_localization;
  visualization_msgs::Marker marker_raw, marker_localization;
  PointCloudXYZPtr map_pcPtr;
  pcl::VoxelGrid<pcl::PointXYZ> vg; // Voxel grid downsampling
  //pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg; // Voxel grid downsampling, seems bad
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // ICP object
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt; // NDT object
  void callback_pc(const sensor_msgs::PointCloud2ConstPtr&);
  void callback_gps(const sensor_msgs::NavSatFixConstPtr&);
  void publishData(PointCloudXYZ, ros::Time, Eigen::Matrix4f);
 public:
  Localization(ros::NodeHandle nh, ros::NodeHandle pnh);
  bool getStatus(void) {return status;}
  void publishMapPC(void);
  void processData(void);
  
};

/*
  __  __      _      ___   _   _ 
 |  \/  |    / \    |_ _| | \ | |
 | |\/| |   / _ \    | |  |  \| |
 | |  | |  / ___ \   | |  | |\  |
 |_|  |_| /_/   \_\ |___| |_| \_|
                                 
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh, pnh("~");
  Localization foo(nh, pnh);
  std::thread myThread(&Localization::publishMapPC, &foo);
  std::thread processThread(&Localization::processData, &foo);
  while(!foo.getStatus()) {ros::spinOnce();}
  // Wait until thread terminated
  myThread.join(); 
  processThread.join();
  return 0;
}

/*

   ____   _          _      ____    ____      ___   __  __   ____    _     
  / ___| | |        / \    / ___|  / ___|    |_ _| |  \/  | |  _ \  | |    
 | |     | |       / _ \   \___ \  \___ \     | |  | |\/| | | |_) | | |    
 | |___  | |___   / ___ \   ___) |  ___) |    | |  | |  | | |  __/  | |___ 
  \____| |_____| /_/   \_\ |____/  |____/    |___| |_|  |_| |_|     |_____|
                                                                           

*/

Localization::Localization(ros::NodeHandle nh, ros::NodeHandle pnh): 
  nh_(nh), pnh_(pnh), counter(0), status(false), removeGround(true), 
  removeOutlier(false), passThrough(true), skip_mode(false), skip_counter(0),
  earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()),
  proj(INIT_LAT, INIT_LON, INIT_ALT, earth)
{
  // Get parameters
  if(!pnh_.getParam("skip_num", skip_num)) skip_num = 0; ROS_INFO("skip_num: %d", skip_num);
  if(!pnh_.getParam("rotate_time", rotate_time)) rotate_time = 16; ROS_INFO("rotate_time: %d", rotate_time);
  if(!pnh_.getParam("radius", radius)) radius = 30.0f; ROS_INFO("radius: %f", radius);
  if(!pnh_.getParam("length", length)) length = 100.0f; ROS_INFO("length: %f", length);
  if(!pnh_.getParam("use_icp", use_icp)) use_icp = true; ROS_INFO("use_icp: %s", (use_icp?"true":"false"));
  if(!pnh_.getParam("save_pcd", save_pcd)) save_pcd = false; ROS_INFO("save_pcd: %s", (save_pcd?"true":"false"));
  if(!pnh_.getParam("filename", filename)) filename = "data"; ROS_INFO("filename: %s", filename.c_str());
  
  writeFile.setFileName(package_path+"/"+ filename +".csv");
  if(save_pcd){ // Save pcd for further usage
    boost::filesystem::path direct(package_path+"/"+"scan");
    if(!boost::filesystem::exists(direct))  boost::filesystem::create_directory(direct);
  }
  guess = Eigen::Matrix4f::Identity();
  vg.setLeafSize(VG_SIZE, VG_SIZE, VG_SIZE);
  if(use_icp){
    icp.setMaximumIterations(MAX_ITER);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(1e-9);
  } else{
    ndt.setMaximumIterations(MAX_ITER);
    ndt.setTransformationEpsilon(1e-5);
    ndt.setEuclideanFitnessEpsilon(1e-5);
    ndt.setStepSize(3.0f); // XXX what is this parameter means?
    ndt.setResolution(1.0f); // XXX what is this parameter means?
  }
  map_pcPtr = PointCloudXYZPtr (new PointCloudXYZ);
  PointCloudXYZPtr tempPtr (new PointCloudXYZ);
  for(auto submap: pcd_name_vec){
    std::string file_path = package_path + "/map" +"/" + submap;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *tempPtr) == -1){
      ROS_ERROR("Cannot find %s, aborting...", file_path.c_str()); ros::shutdown(); return;
    } *map_pcPtr += *tempPtr;
  } ROS_INFO("Successfully load all map, there are %d points", (int)map_pcPtr->points.size());
  if(save_pcd) {pcl::io::savePCDFileBinary(package_path+"/scan/"+"map.pcd", *map_pcPtr); ROS_INFO("Map saved.");}
  ROS_INFO("Map downsampling..."); vg.setInputCloud(map_pcPtr); vg.filter(*map_pcPtr);
  ROS_INFO("After downsampling, there are %d points", (int)map_pcPtr->points.size());
  sub_pc = nh_.subscribe("points_raw", 1, &Localization::callback_pc, this);
  sub_gps = nh_.subscribe("fix", 1, &Localization::callback_gps, this);
  pub_map_pc = pnh_.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
  pub_local_scan = pnh_.advertise<sensor_msgs::PointCloud2>("local_scan", 1);
  pub_raw = pnh_.advertise<visualization_msgs::Marker>("raw_data", 1);
  pub_localization = pnh_.advertise<visualization_msgs::Marker>("localization", 1);
  // Marker initialization
  // Raw data in green
  marker_raw.header.frame_id = FRAME;
  marker_raw.type = visualization_msgs::Marker::LINE_STRIP;
  marker_raw.action = visualization_msgs::Marker::ADD;
  marker_raw.color.g = marker_raw.color.a = 1.0f;
  marker_raw.pose.orientation.w = 1.0f;
  marker_raw.scale.x = 0.3f;
  // ----------------------------------------------------------
  // Result in yellow
  marker_localization.header.frame_id = FRAME;
  marker_localization.type = visualization_msgs::Marker::LINE_STRIP;
  marker_localization.action = visualization_msgs::Marker::ADD;
  marker_localization.color.r = marker_localization.color.g = marker_localization.color.a = 1.0f;
  marker_localization.pose.orientation.w = 1.0f;
  marker_localization.scale.x = 0.5f;
}

void Localization::publishMapPC(void){
  ros::Rate r(FREQ);
  sensor_msgs::PointCloud2 pcout;
  pcl::toROSMsg(*map_pcPtr, pcout);
  pcout.header.frame_id = FRAME;
  while(ros::ok()){
    pub_map_pc.publish(pcout);
    r.sleep();
  }
}

void Localization::callback_gps(const sensor_msgs::NavSatFixConstPtr &gpsPtr){
  proj.Forward(gpsPtr->latitude, gpsPtr->longitude, gpsPtr->altitude, rough_x, rough_y, rough_z);
  geometry_msgs::Point p;
  p.x = rough_x; p.y = rough_y; p.z = rough_z;
  marker_raw.points.push_back(p);
  if(counter==0) ROS_INFO("First GPS data| X: %f| Y: %f| Z:%f", rough_x, rough_y, rough_z);
  ++counter;
  pub_raw.publish(marker_raw);
}

void Localization::callback_pc(const sensor_msgs::PointCloud2ConstPtr &pcPtr){
  if(counter==0) return; // No GPS data receive yet
  PointCloudXYZPtr scan (new PointCloudXYZ);
  pcl::fromROSMsg(*pcPtr, *scan);
  if(save_pcd){
    static int pc_counter = 0;
    pcl::io::savePCDFileASCII(package_path+"/"+"scan/"+std::to_string(pc_counter)+".pcd", *scan);
    ++pc_counter;
  }
  vg.setInputCloud(scan);
  vg.filter(*scan);
  // Extract cloud using distance filter from local scan
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(scan);
  kdtree.radiusSearch(origin, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  boost::shared_ptr<std::vector<int>> indicesptr (new std::vector<int> (pointIdxRadiusSearch)); // Convert to Point Indices
  pcl::ExtractIndices<pcl::PointXYZ> indiceFilter(true);
  indiceFilter.setInputCloud(scan);
  indiceFilter.setIndices(indicesptr);
  indiceFilter.filter(*scan);
  if(removeGround){ // remove ground
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
    // Optional
    seg_plane.setOptimizeCoefficients(true);
    // Mandatory
    seg_plane.setModelType(pcl::SACMODEL_PLANE);
    seg_plane.setMethodType(pcl::SAC_RANSAC);
    seg_plane.setDistanceThreshold(0.3f);
    seg_plane.setInputCloud(scan);
    seg_plane.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> indiceFilter(true); // Initializing with true will allow us to extract the removed indices
    indiceFilter.setInputCloud(scan);
    indiceFilter.setIndices(inliers);
    indiceFilter.setNegative(true);
    indiceFilter.filter(*scan);
  }
  if(removeOutlier){ // remove outlier
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(scan);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*scan);
  }
  std::vector<double> gpsData{rough_x, rough_y, rough_z};
  MyTuple tuple = std::make_tuple(gpsData, *scan, pcPtr->header.stamp);
  myVector.push_back(tuple);
}

void Localization::publishData(PointCloudXYZ icp_result, ros::Time stamp, Eigen::Matrix4f transformation){
  PointCloudXYZRGB resultPlot;
  pcl::copyPointCloud(icp_result, resultPlot);
  for(auto p=resultPlot.points.begin(); p!=resultPlot.points.end(); ++p) p->r = 255;
  // Publish registration result
  sensor_msgs::PointCloud2 pcout;
  pcl::toROSMsg(resultPlot, pcout);
  pcout.header.frame_id = FRAME;
  pub_local_scan.publish(pcout);
  writeFile << stamp
            << transformation(0, 3)
            << transformation(1, 3) 
            << transformation(2, 3) << endrow;
  geometry_msgs::Point p; p.x = transformation(0, 3), p.y = transformation(1, 3), p.z = transformation(2, 3);
  ROS_INFO("Time: %f|X: %f|Y: %f|Z: %f|", stamp.toSec(), p.x, p.y, p.z);
  marker_localization.points.push_back(p);
  pub_localization.publish(marker_localization);
}

void Localization::processData(void){
  static int thread_count = 0;
  std::vector<double> last_gps; last_gps.resize(3);
  std::cout << "In thread: " << std::this_thread::get_id() << "\n";
  while(myVector.size()==0 and ros::ok()) {
    ROS_INFO("No data received yet!"); 
    ros::Duration(1.0).sleep();
  } // Loop to wait until queue has data
  ROS_INFO("Start processing!"); ros::Time total_time = ros::Time::now();
  while(myVector.size()!=0 and ros::ok()){ // Start processing
    ros::Time now = ros::Time::now();
    double score; // Registration score
    PointCloudXYZPtr result (new PointCloudXYZ);
    auto tuple = myVector.front();
    std::vector<double> position = std::get<0>(tuple);
    PointCloudXYZ scan = std::get<1>(tuple);
    ros::Time corresponding_stamp = std::get<2>(tuple);
    if(skip_mode and skip_counter!=skip_num and skip_num!=0 and myVector.size()>skip_num){
      ++skip_counter;
      myVector.erase(myVector.begin()); ++thread_count; position.clear(); scan.clear(); 
      continue;
    } else if (skip_mode) skip_counter=0;
    if(passThrough){
      double upper_x, lower_x, upper_y, lower_y;
      if(thread_count==0) {
        upper_x = position[0]+length/2; lower_x = position[0]-length/2; 
        upper_y = position[1]+length/2; lower_y = position[1]-length/2;
      } else{
        upper_x = guess(0, 3)+length/2; lower_x = guess(0, 3)-length/2; 
        upper_y = guess(1, 3)+length/2; lower_y = guess(1, 3)-length/2;
      }
      // Build local map using pass through
      PointCloudXYZ sub_map;
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(map_pcPtr);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(lower_x, upper_x);
      pass.filter(sub_map);
      pass.setInputCloud(sub_map.makeShared());
      pass.setFilterFieldName("y");
      pass.setFilterLimits(lower_y, upper_y);
      use_icp?(icp.setInputTarget(sub_map.makeShared())):(ndt.setInputTarget(sub_map.makeShared()));
    } else use_icp?(icp.setInputTarget(map_pcPtr)):(ndt.setInputTarget(map_pcPtr)); // Whole map
    if(thread_count==0){ // First scan, which is most important
      // Rotate \rotate_time\ times to get best initial guess (including position and orientation)
      std::vector<double> registration_score; registration_score.resize(rotate_time); // Output score placeholder
      std::vector<Eigen::Matrix4f> registration_res; registration_res.resize(rotate_time); // Output transform placeholder
      Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
      temp(0, 3) = position[0]; temp(1, 3) = position[1]; // GPS position X and Y
      last_x = position[0]; last_y = position[1]; // Put into last data placeholder
      ROS_INFO("Try rotating scan to find best initial guess...");
      for(int i=0; i<rotate_time; ++i){
        double theta = i*(2*M_PI)/rotate_time;
        // Rotate Z-axis
        temp(0, 0) = cos(theta); temp(0, 1) = -sin(theta);
        temp(1, 0) = sin(theta); temp(1, 1) = cos(theta);
        if(use_icp){
          icp.setInputSource(scan.makeShared());
          icp.align(*result, temp);
          registration_score[i] = icp.getFitnessScore(); registration_res[i] = icp.getFinalTransformation();
        } else{
          ndt.setInputSource(scan.makeShared());
          ndt.align(*result, temp);
          registration_score[i] = ndt.getFitnessScore(); registration_res[i] = ndt.getFinalTransformation();
        } // End if-else
        ROS_INFO("|[%2d/%d]| theta: %4.f |score: %f|", i+1, rotate_time, rad2deg(theta), registration_score[i]);
      } // End for
      int min_index; // Find minimum in vector
      std::vector<double>::iterator min_it = std::min_element(registration_score.begin(), registration_score.end());
      min_index = std::distance(registration_score.begin(), min_it);
      ROS_INFO("Min score for registration: %f -> %.f (deg)", registration_score[min_index], rad2deg(min_index*(2*M_PI)/rotate_time));
      guess = registration_res[min_index]; // Put best transformation into guess for iteration
      pcl::transformPointCloud(scan, *result, guess);
      score = registration_score[min_index];
      publishData(*result, corresponding_stamp, guess);
      position.clear(); scan.clear();
      myVector.erase(myVector.begin()); ++thread_count; // Erase first element and advence to next one
      double until_now = (ros::Time::now()-total_time).toSec();
      time_offset = until_now;
      ROS_INFO("Pop one item, totally %d items popped, %d remain! [After processing: %f seconds] [%f per frame]", 
               thread_count, (int)myVector.size(), until_now, until_now/thread_count); 
      continue;
    } // End if(thread_count==0) 
    ros::Time registrationTimer = ros::Time::now();
    // Roughly guess toward the vector provided by GPS
    if(position.size()==3){ // Weird but may occur that size is 0
      guess(0, 3) += position[0] - last_x; last_x = position[0]; 
      guess(1, 3) += position[1] - last_y; last_y = position[1];
    }
    if(use_icp){
      icp.setInputSource(scan.makeShared());
      icp.align(*result, guess);
      score = icp.getFitnessScore();
      guess = icp.getFinalTransformation();
    } else{
      ndt.setInputSource(scan.makeShared());
      ndt.align(*result, guess);
      score = ndt.getFitnessScore();
      guess = ndt.getFinalTransformation();
    }  ROS_INFO("%s takes time [%f] seconds", (use_icp?"ICP":"NDT"), (ros::Time::now()-registrationTimer).toSec());
    if(score<CONVERGE_SCORE and skip_mode==false){
      // Loose the converge criteria
      if(use_icp){
        icp.setTransformationEpsilon(1e-3);
        icp.setEuclideanFitnessEpsilon(1e-3);
      } else{
        ndt.setTransformationEpsilon(1e-3);
        ndt.setEuclideanFitnessEpsilon(1e-3);
      } if(skip_num!=0){
        ROS_INFO("\033[1;33mClose to ground truth, turn on skip mode to speed up processing...\033[0m");
        skip_mode = true;
      }
    }
    publishData(*result, corresponding_stamp, guess);
    double until_now = (ros::Time::now()-total_time).toSec() - time_offset;
    position.clear(); scan.clear(); myVector.erase(myVector.begin()); ++thread_count; // Erase first element and advance to next one
    ROS_INFO("Pop one item, totally %d items popped, %d remain! [After processing: %f seconds] [%f per frame]", 
             thread_count, (int)myVector.size(), until_now+time_offset, until_now/(thread_count-1));
  }
  ROS_INFO("Finish!");
  status = true; exit(EXIT_SUCCESS); 
}