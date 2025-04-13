#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

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

#include "ros2_rs_pcl/filter/rspcl_filter_component.hpp"

RspclFilterComponent::RspclFilterComponent() : Node("pclsub")
{
  // Create TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", 
    10, 
    std::bind(&RspclFilterComponent::timer_callback, this, std::placeholders::_1)
  );

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_filtered", 10);
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Example PassThrough filter on Z-axis
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(pcl_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.3, 0.1);
  pass.filter(*cloud_filtered);

  // Example CropBox filter
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(cloud_filtered);
  crop.setMin(Eigen::Vector4f(-0.3, -0.9, -1.4, 1.0));
  crop.setMax(Eigen::Vector4f(0.3, 0.1, 1.0, 1.0));
  crop.setNegative(true);
  crop.filter(*cloud_filtered);

  // Convert back to ROS message
  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  sensor_msg.header.stamp = this->get_clock()->now();
  // Ensure the frame_id is now hesai_lidar
  sensor_msg.header.frame_id = "hesai_lidar";

  RCLCPP_INFO(this->get_logger(), "Publishing filtered cloud in hesai_lidar frame");
  publisher_->publish(sensor_msg);
}
