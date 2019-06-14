#include "track.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_node");
  ros::NodeHandle nh, pnh("~");
  Track foo(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}

