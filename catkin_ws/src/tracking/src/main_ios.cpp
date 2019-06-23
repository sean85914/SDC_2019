#include "track_ios.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_ios_node");
  ros::NodeHandle nh, pnh("~");
  Track foo(nh, pnh);
  thread processThread(&Track::process_data, &foo);
  thread checkThread(&Track::check_bag_has_end, &foo);
  while(!foo.getStatus()) ros::spinOnce();
  processThread.join();
  return 0;
}

