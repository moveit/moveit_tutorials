#include "ros/ros.h"
#include "image_transport/image_transport.h"

void filteredDepthCallback(const sensor_msgs::ImageConstPtr& msg){
  // ROS_INFO("Filtered Depth Callback");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("filtered/depth", 10, filteredDepthCallback);

  ros::spin();
  return 0;
}
