#include "ros/ros.h"
#include "beginner_tutorials/TurtleWrite.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_write_client");
  if (argc != 3)
  {
    ROS_INFO("usage: turtle_write_client message scale");
    return -1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::TurtleWrite>("turtle_write");
  beginner_tutorials::TurtleWrite srv;
  srv.request.message = argv[1];
  srv.request.scale = atof(argv[2]);
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
