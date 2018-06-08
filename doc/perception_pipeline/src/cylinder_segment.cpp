#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

void addCylinder(std::vector<double>* cylinder_params)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // BEGIN_SUB_TUTORIAL add_cylinder
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "camera_rgb_optical_frame";
  collision_object.id = "cylinder";

  // Define a cylinder which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  // height of cylinder
  primitive.dimensions[0] = cylinder_params->at(7);
  // radius of cylinder
  primitive.dimensions[1] = cylinder_params->at(0);

  // Define a pose for the cylinder (specified relative to frame_id)
  geometry_msgs::Pose cylinder_pose;
  // Computing and setting quaternion from axis angle representation
  Eigen::Vector3d v(cylinder_params->at(1), cylinder_params->at(2), cylinder_params->at(3));
  Eigen::Vector3d u(0., 0., 1.);
  Eigen::Vector3d axis;
  double angle;
  axis = u.cross(v);
  axis.normalize();
  angle = acos(v.dot(u));
  cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
  cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
  cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
  cylinder_pose.orientation.w = cos(angle / 2);
  // Set the position of cylinder
  cylinder_pose.position.x = cylinder_params->at(4);
  cylinder_pose.position.y = cylinder_params->at(5);
  cylinder_pose.position.z = cylinder_params->at(6);

  // Add cylinder as collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);
  // END_SUB_TUTORIAL
}

void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, std::vector<double>* cylinder_params)
{
  double max_angle_rady = 0.0;
  double min_angle_rady = std::numeric_limits<double>::infinity();

  double bot[3];
  double top[3];
  // BEGIN_SUB_TUTORIAL extract_location_height
  // Consider a point inside the point cloud and imagaine that point is formed on a XY plane where the perpendicular distance from the plane to the camera is Z.
  // The perpendicular drawn from the camera to the plane hits at center of the XY plane 
  // We have the x and y coordinate of the point which is formed on the XY plane.
  // X is the horizontal axis and Y is the vertical axis
  // C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane
  // Now we know Z is the perpendicular distance from the point to the camera. 
  // If you need to find the  actual distance d from the point to the camera, you shloud calculate the hypotenuse hypot(pt.z, pt.x)
  // angle the point made horizontally atan2(pt.z,pt.x);
  // angle the point made Verticlly    atan2(pt.z, pt.y);

  // Iterate trough all the points in the filtered point cloud
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, input->points)
  {
    // Find the coordinates of the highest point
    if (atan2(pt.z, pt.y) < min_angle_rady)
    {
      min_angle_rady = atan2(pt.z, pt.y);
      bot[0] = pt.x;
      bot[1] = pt.y;
      bot[2] = pt.z;
    }
    // Find the coordinates of the lowest point
    else if (atan2(pt.z, pt.y) > max_angle_rady)
    {
      max_angle_rady = atan2(pt.z, pt.y);
      top[0] = pt.x;
      top[1] = pt.y;
      top[2] = pt.z;
    }
  }
  // Store the centre point of cylinder
  cylinder_params->push_back((top[0] + bot[0]) / 2);
  cylinder_params->push_back((top[1] + bot[1]) / 2);
  cylinder_params->push_back((top[2] + bot[2]) / 2);
  // Store the height of cylinder
  cylinder_params->push_back(sqrt(pow((bot[0] - top[0]), 2) + pow((bot[1] - top[1]), 2) + pow((bot[2] - top[2]), 2)));
  // END_SUB_TUTORIAL
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

void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder,
                     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
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
  // Convert to pcl::PointXYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);
  // Using passthough filter to get roi
  passThroughFilter(cloud);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  computeNormals(cloud, cloud_normals);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  removePlaneSurface(cloud, inliers_plane);
  extractNormals(cloud_normals, inliers_plane);
  // BEGIN_SUB_TUTORIAL extract_cylinder
  // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length. 
  // It has a public attribute values of type std::vector< float >. 
  // values[0-2] hold a point on the center line of the cylinder. 
  // values[3-5] hold direction vector of the z-axis. 
  // values[6] is the radius of the cylinder. 
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  // Extract the cylinder using SACSegmentation. 
  extractCylinder(cloud, coefficients_cylinder, cloud_normals);
  // END_SUB_TUTORIAL
  if (cloud->points.empty())
  {
    ROS_ERROR_STREAM("Can't find the cylindrical component.");
  }
  else
  {
    static bool points_not_found = true;
    if (points_not_found)
    {
      // BEGIN_TUTORIAL
      // CALL_SUB_TUTORIAL extract_cylinder
      // Hold the relavent parameter of cylinder needed for creating a collision object
      // There are a total of 7 parameters:
      // 0 - radius of cylinder.
      // 1-3 - direction vector of z-axis.
      // 4-6 - centre point of the cylinder.
      // 7 - height of the cylinder.
      std::vector<double> cylinder_params;
      // radius of cylinder
      cylinder_params.push_back(coefficients_cylinder->values[6]);
      // i,j,k values of the direction vector of z-axis
      cylinder_params.push_back(coefficients_cylinder->values[3]);
      cylinder_params.push_back(coefficients_cylinder->values[4]);
      cylinder_params.push_back(coefficients_cylinder->values[5]);
      // Compute the Centre point of the cylinder using standard geometry
      extractLocationHeight(cloud, &cylinder_params);
      // CALL_SUB_TUTORIAL extract_location_height
      // Use the parameters extracted to add the cylinder to the planning scene as a collision object.
      addCylinder(&cylinder_params);
      // CALL_SUB_TUTORIAL add_cylinder
      // END_TUTORIAL
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