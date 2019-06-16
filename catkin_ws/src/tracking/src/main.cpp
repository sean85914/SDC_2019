#include "track.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_node");
  ros::NodeHandle nh, pnh("~");
  Track foo(nh, pnh);
  thread processThread(&Track::process_data, &foo);
  while(!foo.getStatus()) ros::spinOnce();
  processThread.join();
  return 0;
}

