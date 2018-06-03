/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Ridhwan Luthra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ridhwan Luthra */

#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_publisher_maintain_time");
  ros::NodeHandle nh;

  ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
  ros::Rate loop_rate(0.1);

  // Variable holding the rosbag containing point cloud data.
  rosbag::Bag bagfile;
  std::string path = ros::package::getPath("moveit_tutorials");
  path += "/doc/perception_pipeline/bags/perception_tutorial.bag";
  bagfile.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/camera/depth_registered/points");

  // Iterator for topics in bag.
  rosbag::View bagtopics_iter(bagfile, rosbag::TopicQuery(topics));

  for (auto const msg : bagtopics_iter)
  {
    sensor_msgs::PointCloud2::Ptr point_cloud_ptr = msg.instantiate<sensor_msgs::PointCloud2>();
    if (point_cloud_ptr == NULL)
    {
      std::cout << "error" << std::endl;
      break;
    }

    while (ros::ok())
    {
      point_cloud_ptr->header.stamp = ros::Time::now();
      point_cloud_publisher.publish(*point_cloud_ptr);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  bagfile.close();
  return 0;
}
