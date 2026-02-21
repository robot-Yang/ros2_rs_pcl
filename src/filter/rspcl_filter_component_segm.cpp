#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>

// Include TF libraries
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// Include your filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "ros2_rs_pcl/filter/rspcl_filter_component.hpp"

// Make composable node
#include "rclcpp_components/register_node_macro.hpp"

RspclFilterComponent::RspclFilterComponent(const rclcpp::NodeOptions & options) : Node("pclsub")
{
  // Create TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 
    10, 
    std::bind(&RspclFilterComponent::timer_callback, this, std::placeholders::_1)
  );
  publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_RAM_presegm", 10);
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_RAM_segm", 10);
}

void RspclFilterComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  // Make sure we have transforms available before proceeding
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    // Lookup the transform from livox_lidar frame to hesai_lidar frame
    transform_stamped = tf_buffer_->lookupTransform(
        "hesai_lidar",        // target frame
        cloud_msg->header.frame_id, // source frame (likely "livox_lidar")
        tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  // Transform the incoming point cloud to hesai_lidar frame
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);

  // Now we have the cloud in hesai_lidar frame, proceed with filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);

  // Example PassThrough filter on Z-axis
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.2 , -0.6);
  //pass.setFilterLimits(-1.3, 0.8);  //Original limits
  pass.filter(*cloud_filtered);


  // Example PassThrough filter on X-axis
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-2.5, -0.6);
  pass.filter(*cloud_filtered);

  // Example PassThrough filter on Y-axis
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1, 1);
  pass.filter(*cloud_filtered);
  

  // // Example CropBox filter
  // pcl::CropBox<pcl::PointXYZ> crop;
  // crop.setInputCloud(cloud_filtered);
  // crop.setMin(Eigen::Vector4f(-0.3, -0.9, -1.4, 1.0));
  // crop.setMax(Eigen::Vector4f(0.3, 0.1, 1.0, 1.0));
  // crop.setNegative(true);
  // crop.filter(*cloud_filtered);


  // Estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_filtered);
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(20);
  ne.compute(*cloud_normals);

  // Set up SACSegmentationFromNormals for horizontal planes
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.05);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);

  // Only accept planes with normals close to Z axis (horizontal)
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(pcl::deg2rad(10.0)); // 15 degree tolerance

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.segment(*inliers, *coefficients);

  
  if (inliers->indices.empty()) {
    RCLCPP_WARN(get_logger(), "No horizontal plane found.");
    return;
  }


  // Extract only the horizontal plane points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_segmented);

  // Obtain the closest cluster
  cloud_segmented=euclideanClusterExtraction(cloud_segmented);

  // Convert pre segmented pc back to ROS message
  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  sensor_msg.header.stamp = this->get_clock()->now();
  // Ensure the frame_id is now hesai_lidar
  sensor_msg.header.frame_id = "hesai_lidar";

  //RCLCPP_INFO(this->get_logger(), "Publishing filtered cloud in hesai_lidar frame");
  publisher2_->publish(sensor_msg);

  // Convert back to ROS message
  sensor_msgs::msg::PointCloud2 sensor_msg2;
  pcl::toROSMsg(*cloud_segmented, sensor_msg2);
  sensor_msg2.header.stamp = this->get_clock()->now();
  // Ensure the frame_id is now hesai_lidar
  sensor_msg2.header.frame_id = "hesai_lidar";

  //RCLCPP_INFO(this->get_logger(), "Publishing filtered cloud in hesai_lidar frame");
  publisher_->publish(sensor_msg2);
}



pcl::PointCloud<pcl::PointXYZ>::Ptr
RspclFilterComponent::euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); // 2cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // We'll pick the cluster whose centroid is closest to the origin
  float min_ang = std::numeric_limits<float>::max();
  float max_size = 0.0f;
  pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cluster(new pcl::PointCloud<pcl::PointXYZ>());

  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud)[idx]);
    }

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    float angle = std::abs(centroid[0]/centroid[1]);

    // Keep the cluster that is closest
    if (angle < min_ang)
    {
      min_ang= angle;
      *closest_cluster = *cloud_cluster;
    }
  }

  return closest_cluster;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RspclFilterComponent)