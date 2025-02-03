// rspcl_filter_component.hpp
#ifndef RSPCL_FILTER_COMPONENT_HPP_
#define RSPCL_FILTER_COMPONENT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RspclFilterComponent : public rclcpp::Node {
public:
  RspclFilterComponent();

private:
  void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  // Declare the TF buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
};

#endif // RSPCL_FILTER_COMPONENT_HPP_
