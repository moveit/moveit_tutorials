/*
*
* Project Name:   Visual perception for the visually impaired
* Author List:    Pankaj Baranwal, Ridhwan Luthra, Shreyas Sachan, Shashwat Yashaswi
* Filename:     cluster_extraction.cpp
* Functions:    cloud_cb, main 
* Global Variables: pub -> Ros publisher
*
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <stdio.h>

using namespace::std;

ros::Publisher pub;
/*
*
* Function Name: cloud_cb
* Input: input -> A ros message service that provides point cloud data from kinect
* Output:  Publishes extracted cluster. 
* Logic:   first a voxelgrid filter is applied to make the cloud less dense.
*          then Sac segmentation is done using ransac model to extract the planes
*          Then using extractIndices the point cloud without the floor plane is extracted.
*          then eucledian clustering is executed to find clusters
*          the various clusters are then concatinated and then published, this also removes outliers
* Example Call: Callback function. Manual calling not required. 
*
*/
void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{  
  // this is the voxel grid filtering
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PCLPointCloud2 cloud_voxeled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_voxeled);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (cloud_voxeled, *cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while 30% of the cloud is there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  // create object and set params for clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (250);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);


  pcl::PCLPointCloud2 cloud_final;
  pcl::PCLPointCloud2 temp;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
  for (int it = 0; it < cluster_indices.size(); ++it)
  {
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> ext;
    ext.setInputCloud (cloud_filtered);
    ext.setIndices(boost::shared_ptr<pcl::PointIndices> (new pcl::PointIndices(cluster_indices[it])));
    ext.setNegative (false);
    ext.filter (cloud_xyzrgb);

    pcl::toPCLPointCloud2 (cloud_xyzrgb, temp);
    pcl::concatenatePointCloud(cloud_final, temp, cloud_final);
  }
  pub.publish (cloud_final);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("just_clusters", 1);

  // Spin
  ros::spin ();
}