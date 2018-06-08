#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

void add_object(std::vector<double>* cylinder_params)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "camera_rgb_optical_frame";
  // The id of the object is used to identify it.
  collision_object.id = "cylinder1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = cylinder_params->at(7);
  primitive.dimensions[1] = cylinder_params->at(0);

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  Eigen::Vector3d v(cylinder_params->at(1), cylinder_params->at(2), cylinder_params->at(3));
  Eigen::Vector3d u(0., 0., 1.);
  Eigen::Vector3d axis;
  double angle;
  axis = u.cross(v);
  axis.normalize();
  angle = acos(v.dot(u));
  box_pose.orientation.x = axis.x() * sin(angle / 2);
  box_pose.orientation.y = axis.y() * sin(angle / 2);
  box_pose.orientation.z = axis.z() * sin(angle / 2);
  box_pose.orientation.w = cos(angle / 2);

  box_pose.position.x = cylinder_params->at(4);
  box_pose.position.y = cylinder_params->at(5);
  box_pose.position.z = cylinder_params->at(6);

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void get_coords(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, std::vector<double>* cylinder_params)
{
  double max_angle_rady = 0.0;
  double min_angle_rady = std::numeric_limits<double>::infinity();

  double bot[3];
  double top[3];
  
  //to iterate trough all the points in the filtered point cloud 
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, input->points)
  {
    // std::cout<<i++<<endl;
    if(atan2(pt.z, pt.y) < min_angle_rady)
    {
      // keep updating min angle point
      min_angle_rady=atan2(pt.z, pt.y);
      bot[0] = pt.x;
      bot[1] = pt.y;
      bot[2] = pt.z;
    }
    else if(atan2(pt.z, pt.y) > max_angle_rady)
    {
      // keep updating the max angle point
      max_angle_rady = atan2(pt.z, pt.y);
      top[0] = pt.x;
      top[1] = pt.y;
      top[2] = pt.z;
    }
  }
  cylinder_params->push_back((top[0] + bot[0]) / 2);
  cylinder_params->push_back((top[1] + bot[1]) / 2);
  cylinder_params->push_back((top[2] + bot[2]) / 2);
  cylinder_params->push_back(sqrt(pow((bot[0] - top[0]), 2) + pow((bot[1] - top[1]), 2) + pow((bot[2] - top[2]), 2)));
}

void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // min and max values in z axis to keep
  pass.setFilterLimits(0.3, 1.1);
  pass.filter(*cloud);
}

void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
{
  // create a SAC segmentor without using normals
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // run at max 1000 iterations before giving up
  seg.setMaxIterations(1000);
  // tolerence for variation from model 
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  // Create the segmentation object for the planar model and set all the parameters
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  seg.segment(*inliers_plane, *coefficients_plane);
  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_plane);
  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud);
}

void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  seg.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  seg.setMaxIterations(10000);
  // tolerence for variation from model 
  seg.setDistanceThreshold(0.05);
  // min max values of radius in meters to consider
  seg.setRadiusLimits(0, 1);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

  // Extract the cylinder inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(*cloud);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  passThroughFilter(cloud);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  computeNormals(cloud, cloud_normals);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  removePlaneSurface(cloud, inliers_plane);
  extractNormals(cloud_normals, inliers_plane);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  extractCylinder(cloud, coefficients_cylinder, cloud_normals);
  if (cloud->points.empty())
  {
    ROS_ERROR_STREAM("Can't find the cylindrical component.");
  }
  else
  {
    static bool points_not_found = true;
    if (points_not_found)
    {
      std::vector<double> cylinder_params;
      // radius of cylinder
      cylinder_params.push_back(coefficients_cylinder->values[6]);
      // i,j,k values of the direction vector of z-axis
      cylinder_params.push_back(coefficients_cylinder->values[3]);
      cylinder_params.push_back(coefficients_cylinder->values[4]);
      cylinder_params.push_back(coefficients_cylinder->values[5]);
      get_coords(cloud, &cylinder_params);
      // for (auto i: cylinder_params)
      //   std::cout << i << ' ';
      // std::cout<<"\n";
      add_object(&cylinder_params);
      points_not_found = false;
    }
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;
  // Initialize subscriber to the raw point cloud topic
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);
  // Spin
  ros::spin();
}