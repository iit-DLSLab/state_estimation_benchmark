#include "state_estimator/state_estimator_node.hpp"
#include <signal.h>


void mySigintHandler(int sig);

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

int main(int argc, char **argv) {
  ROS_INFO("Starting state estimator");
  ros::init(argc, argv, "state_estimator",ros::init_options::NoSigintHandler);

  signal(SIGINT, mySigintHandler);

  ros::NodeHandle nh("~");
  state_estimator::state_estimator_node node(nh);

  while (!g_request_shutdown) {
    ros::spinOnce();
    boost::this_thread::sleep_for(boost::chrono::microseconds(100)); // FIX ME
  }
    
  ros::shutdown(); //Due to a weird bug, ROS must shutdown before the node shuts down
  node.shutdown();
  
 
  return 0;
}

void mySigintHandler(int sig) {
    g_request_shutdown = 1;
}

