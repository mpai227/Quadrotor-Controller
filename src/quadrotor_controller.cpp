#include <ros/ros.h>
#include <quadrotor_controller/QuadrotorController.h>

int main(int argc, char** argv) { 

  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle n("~");

  QuadrotorController q;

  if (!q.initialize(n))
    {
      ROS_ERROR("%s: failed to initialize quadrotor controller",
                ros::this_node::getName().c_str());
      return EXIT_FAILURE;
    }

  ros::spin();

  return EXIT_SUCCESS;
}
