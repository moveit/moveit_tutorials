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
  // 
  // Adding Cylinder to Planning Scene
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "camera_rgb_optical_frame";
  collision_object.id = "cylinder";

  // Define a cylinder which will be added to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  // Setting height of cylinder.
  primitive.dimensions[0] = cylinder_params->at(7);
  // Setting radius of cylinder.
  primitive.dimensions[1] = cylinder_params->at(0);

  // Define a pose for the cylinder (specified relative to frame_id).
  geometry_msgs::Pose cylinder_pose;
  // Computing and setting quaternion from axis angle representation.
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
  // Setting the position of cylinder.
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
  // Consider a point inside the point cloud and imagine that point is formed on a XY plane where the perpendicular distance from the plane to the camera is Z. |br|
  // The perpendicular drawn from the camera to the plane hits at center of the XY plane. |br|
  // We have the x and y coordinate of the point which is formed on the XY plane. |br|
  // X is the horizontal axis and Y is the vertical axis. |br|
  // C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane. |br|
  // Now we know Z is the perpendicular distance from the point to the camera. |br|
  // If you need to find the  actual distance d from the point to the camera, you should calculate the hypotenuse- |code_start| hypot(pt.z, pt.x);\ |code_end| |br|
  // angle the point made horizontally- |code_start| atan2(pt.z,pt.x);\ |code_end| |br|
  // angle the point made Verticlly- |code_start| atan2(pt.z, pt.y);\ |code_end| |br|
  // Loop over the entire pointcloud.
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
  // BEGIN_SUB_TUTORIAL callback
  //
  // Perception Related
  // ^^^^^^^^^^^^^^^^^^
  // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for most of the processing.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud);
  // Using passthough filter to get region of interest. A passthrough filter just eliminates the point cloud values which do not lie in the user specified range.
  passThroughFilter(cloud);
  // Declare normals and call function to compute point normals.
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  computeNormals(cloud, cloud_normals);
  // inliers_plane will hold the indices of the point cloud that correspond to a plane.
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
  removePlaneSurface(cloud, inliers_plane);
  // We had calculated the point normals in a previous call to computeNormals, 
  // now we will be extracting the normals that correspond to the plane on which cylinder lies. 
  // It will be used to extract the cylinder.
  extractNormals(cloud_normals, inliers_plane);
  // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length. 
  // It has a public attribute |code_start| values\ |code_end| of type |code_start| std::vector< float >\ |code_end|\ . |br|
  // |code_start| Values[0-2]\ |code_end| hold a point on the center line of the cylinder. |br|
  // |code_start| Values[3-5]\ |code_end| hold direction vector of the z-axis. |br|
  // |code_start| Values[6]\ |code_end| is the radius of the cylinder.
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
      // CALL_SUB_TUTORIAL callback
      // 
      // Storing Relavant Cylinder Values
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // The information that we have in |code_start| coefficients_cylinder\ |code_end| is not enough to define our cylinder.
      // It does not have the actual location of the cylinder nor the actual height. |br|
      // We define a vector to hold the parameters that are actually needed for defining a collision object completely. |br|
      // There are a total of 7 parameters: |br|
      // 0 - radius of cylinder. |br|
      // 1-3 - direction vector of z-axis. |br|
      // 4-6 - centre point of the cylinder. |br|
      // 7 - height of the cylinder. |br|
      std::vector<double> cylinder_params;
      // Store the radius of the cylinder.
      cylinder_params.push_back(coefficients_cylinder->values[6]);
      // Store direction vector of z-axis of cylinder.
      cylinder_params.push_back(coefficients_cylinder->values[3]);
      cylinder_params.push_back(coefficients_cylinder->values[4]);
      cylinder_params.push_back(coefficients_cylinder->values[5]);
      //
      // Extracting Location and Height
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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