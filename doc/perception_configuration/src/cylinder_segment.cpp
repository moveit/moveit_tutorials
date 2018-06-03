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
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>



// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <stdio.h>

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
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Build a passthrough filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.3, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // pcl::PCLPointCloud2 outcloud;
  // pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  // pub.publish (outcloud);

  // Estimate point normals
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  pcl::SACSegmentation<pcl::PointXYZRGB> sega;
  sega.setOptimizeCoefficients (true);
  sega.setModelType (pcl::SACMODEL_PLANE);
  sega.setMethodType (pcl::SAC_RANSAC);
  sega.setMaxIterations (1000);
  sega.setDistanceThreshold (0.01);
  sega.setInputCloud (cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  // pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  // seg.setNormalDistanceWeight (0.1);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.03);
  // seg.setInputCloud (cloud_filtered);
  // seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  sega.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  // pcl::PCLPointCloud2 outcloud;
  // pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  // pub.publish (outcloud);

  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 1);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

    pcl::PCLPointCloud2 outcloud;
    // pcl::PCLPointCloud2 temp;
    // pcl::toPCLPointCloud2 (*cloud_plane, temp);
    pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud);
    // pcl::concatenatePointCloud(outcloud, temp, outcloud);
    // BEGIN_SUB_TUTORIAL publish_processed_cloud
    // Publishing processed point cloud
    pub.publish (outcloud);
    // END_SUB_TUTORIAL
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster_extraction");
  ros::NodeHandle nh;
  // BEGIN_TUTORIAL
  //
  // Setting Up Subscriber and Publisher
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Initialize subscriber to the raw point cloud topic
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Initialise publisher to publish processed point cloud to new topic
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/processed_points", 1);
  // CALL_SUB_TUTORIAL publish_processed_cloud
  // END_TUTORIAL
  // Spin
  ros::spin ();
}