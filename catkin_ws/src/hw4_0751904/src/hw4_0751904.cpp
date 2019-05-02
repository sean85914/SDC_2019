/*
 *  Plot the odometry data, it is in red for /zed/odom, for fusioned one, it is in green. Only consider 
 *  position data
 *  Subscribe topics:
 *    /zed/odom (nav_msgs/Odometry): raw data from zed
 *    /robot_pose_ekf/odom_combined (geometry_msgs::PoseWithCovarianceStamped): after fusion visual odometry and IMU data
 *  Publish topics:
 *    /visualize/original_odom (visualization_msgs/Marker): corresponding to /zed/odom
 *    /visualize/filtered_odom (visualization_msgs/Marker): corresponding to /robot_pose_ekf/odom_combined
 *  Parameter:
 *    ~debug: if true, record the position and orientation from both topics
 */

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

bool debug;
std::string zed_odom_file = "zed_odom.txt";
std::string filtered_file = "filter.txt";
std::string file_path;
std::fstream file1, file2;

ros::Publisher pub_ori_path,
               pub_filtered_path;
visualization_msgs::Marker original, filtered;

std_msgs::ColorRGBA r,g;

/*
 * Initial marker, including action, type, pose, scale and color
 * bool type: true if original(red), and false otherwise(green)
 */
void initial_marker(visualization_msgs::Marker &msg, bool type){
  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = msg.scale.z = 0.02; // 2 cm
  msg.color = (type==true?r:g);
}
// Callback for /zed/odom
void cb_ori(const nav_msgs::Odometry msg){
  original.header.frame_id = "odom_combined"; // Use "odom_combined" as filtered one to plot in same global frame
  geometry_msgs::Point p;
  p.x = msg.pose.pose.position.x;
  p.y = msg.pose.pose.position.y;
  p.z = msg.pose.pose.position.z;
  original.points.push_back(p);
  pub_ori_path.publish(original);
  if(debug){ // Record position and orientation
    tf::Quaternion quat(msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat(quat);
    double x = msg.pose.pose.position.x,
           y = msg.pose.pose.position.y,
           z = msg.pose.pose.position.z;
    double r_, p_, y_; rot_mat.getRPY(r_, p_, y_);
    file1 << x << " " << y << " " << z << " " << r_ << " " << p_ << " " << y_ << std::endl;
  }
}
// Callback for /robot_pose_ekf/odom_combined
void cb_filtered(const geometry_msgs::PoseWithCovarianceStamped msg){
  filtered.header.frame_id = msg.header.frame_id;
  geometry_msgs::Point p;
  p.x = msg.pose.pose.position.x;
  p.y = msg.pose.pose.position.y;
  p.z = msg.pose.pose.position.z;
  filtered.points.push_back(p);
  pub_filtered_path.publish(filtered);
  if(debug){ // Record position and orientation
    tf::Quaternion quat(msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat(quat);
    double x = msg.pose.pose.position.x,
           y = msg.pose.pose.position.y,
           z = msg.pose.pose.position.z;
    double r_, p_, y_; rot_mat.getRPY(r_, p_, y_);
    file2 << x << " " << y << " " << z << " " << r_ << " " << p_ << " " << y_ << std::endl;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "plot_path_node");
  // Plot color
  r.r = r.a = g.g = g.a = 1.0;
  // Initial markers
  initial_marker(original, true); initial_marker(filtered, false);
  ros::NodeHandle nh, pnh("visualize");
  // Subscribers
  ros::Subscriber sub_ori = nh.subscribe("/zed/odom", 1, cb_ori);
  ros::Subscriber sub_filtered = nh.subscribe("/robot_pose_ekf/odom_combined", 1, cb_filtered);
  // Publisher
  pub_ori_path = pnh.advertise<visualization_msgs::Marker>("original_odom", 1);
  pub_filtered_path = pnh.advertise<visualization_msgs::Marker>("filtered_odom", 1);
  // Get file path
  file_path = ros::package::getPath("hw4_0751904");
  file_path += "/data/";
  zed_odom_file = file_path + zed_odom_file;
  filtered_file = file_path + filtered_file;
  // Get parameter
  ros::param::get("~debug", debug);
  ROS_INFO("Debug set to %s", (debug==true?"true":"false")); if(debug){
    file1.open(zed_odom_file.c_str(), std::ios::out);
    file2.open(filtered_file.c_str(), std::ios::out);
    if(!file1.is_open() or !file2.is_open()){
      ROS_WARN("Cannot open file(s)."); return -1;
    }
  }
  ROS_INFO("Ready to plot...");
  while(ros::ok()) ros::spinOnce();
  // Close files if in debug mode
  if(debug) {file1.close(); file2.close();}
  return 0;
}
