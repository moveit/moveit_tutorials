#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_publisher_maintain_time");
  ros::NodeHandle n;

  ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
  ros::Rate loop_rate(0.1);

  rosbag::Bag bag;
  std::string path = ros::package::getPath("moveit_tutorials");
  path += "/doc/perception_pipeline/bags/perception_tutorial.bag";
  // path += "/doc/perception_pipeline/bags/2018-05-28-07-49-11.bag";
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/camera/depth_registered/points");

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::Ptr p = m.instantiate<sensor_msgs::PointCloud2>();
    if (p != NULL)
    {
      while (ros::ok())
      {
        p->header.stamp = ros::Time::now();
        pc_pub.publish(*p);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else
    {
      std::cout<<"error"<<std::endl;
      break;
    }
  }
  bag.close();
  return 0;
}