// C++ STL
#include <fstream>
// ROS
#include <ros/ros.h>
// TF
#include <tf/tf.h>
// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// MSG
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

typedef struct {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
}OdomData;

bool haveData, have_wo;
int counter = 0;
std::string prefix; // file prefix
std::ofstream f;
OdomData data;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());

void cb(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::fromROSMsg(*msg, *cloud_in);
  haveData = true;
}

void cbCombined(const nav_msgs::OdometryConstPtr& odomPtr, const sensor_msgs::PointCloud2ConstPtr& cloudPtr){
  pcl::fromROSMsg(*cloudPtr, *cloud_in);
  data.x = odomPtr->pose.pose.position.x;
  data.y = odomPtr->pose.pose.position.y;
  data.z = odomPtr->pose.pose.position.z;
  tf::Quaternion quat(odomPtr->pose.pose.orientation.x,
                      odomPtr->pose.pose.orientation.y,
                      odomPtr->pose.pose.orientation.z,
                      odomPtr->pose.pose.orientation.w);
  tf::Matrix3x3 rot_mat(quat);
  rot_mat.getRPY(data.roll, data.pitch, data.yaw);
  haveData = true;
}

void timerCb(const ros::TimerEvent& event){
  if(haveData){
    std::string file_name; file_name.clear(); 
    if(prefix!="") file_name += (prefix + "_");
    file_name += std::to_string(counter) + ".pcd";
    pcl::io::savePCDFileASCII(file_name, *cloud_in);
    ++counter; ROS_INFO("Message received: %d", counter);
    if(have_wo){
      f << data.x << " "
        << data.y << " "
        << data.z << " "
        << data.roll << " "
        << data.pitch << " "
        << data.yaw << "\n";
    }
    haveData = false;
  }
}

int main(int argc, char** argv)
{
  haveData = false;
  ros::init(argc, argv, "save_pcd_node");
  ros::NodeHandle nh, pnh("~");
  ros::Subscriber sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
  // Refer: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
  typedef message_filters::sync_policies::ApproximateTime\
                                   <nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> sync;
  boost::shared_ptr<sync> sync_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  if(!pnh.getParam("prefix", prefix)) prefix=""; ROS_INFO("prefix: %s", prefix.c_str());
  if(!pnh.getParam("have_wo", have_wo)) have_wo = false; ROS_INFO("have_wo: %s", (have_wo==true?"true":"false"));
  if(!have_wo){
    sub = nh.subscribe("velodyne_points", 1, cb);
  }
  else{
    f.open("odometry.txt"); if(!f) {ROS_ERROR("Cannot open file, aborting..."); return -1;}
    odom_sub.subscribe(nh, "/husky_velocity_controller/odom", 1);
    cloud_sub.subscribe(nh, "/velodyne_points", 1);
    sync_.reset(new sync(MySyncPolicy(1), odom_sub, cloud_sub));
    sync_->registerCallback(boost::bind(&cbCombined, _1, _2));
  }
  //ros::Rate r(10); // 10 hz, 0.1 second interval per scan
  ros::Timer timer = pnh.createTimer(ros::Duration(0.1), timerCb);
  while(ros::ok()) ros::spinOnce();
  if(have_wo) f.close(); // Close file
  return 0;
}
