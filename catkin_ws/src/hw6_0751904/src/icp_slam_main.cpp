#include <icp_slam.h>
#include <signal.h>

static ICPSLAM* signal_object;
extern "C" void signal_handler(int signum){ signal_object->mySigintHandler(signum);}

// ************************************************************************************************
// ********************************************* MAIN *********************************************
// ************************************************************************************************
int main(int argc, char** argv)
{
  int mode = atoi(argv[1]);
  assert((mode==0)or(mode==1));
  if(mode==0) ros::init(argc, argv, "icp_slam", ros::init_options::NoSigintHandler);
  else ros::init(argc, argv, "icp_slam");
  ros::NodeHandle nh, pnh("~");
  ICPSLAM foo(nh, pnh, mode);
  signal_object = &foo;
  if(mode==0) signal(SIGINT, signal_handler);
  while(ros::ok()){
    ros::spinOnce(); if(mode) {if(foo.getStatus() != 0) ros::shutdown();}
  }
  return 0;
}
