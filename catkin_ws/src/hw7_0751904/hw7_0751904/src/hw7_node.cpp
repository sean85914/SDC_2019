#include "hw7_class.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw7_node");
  ros::NodeHandle nh, pnh("~");
  HW7 foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
