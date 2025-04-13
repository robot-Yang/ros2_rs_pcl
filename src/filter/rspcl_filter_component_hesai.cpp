#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "ros2_rs_pcl/filter/rspcl_filter_component.hpp"


RspclFilterComponent::RspclFilterComponent() : Node("pclsub")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar_points", // "/hesai_lidar/lidar_points"
    10, 
    std::bind(&RspclFilterComponent::timer_callback, this, std::placeholders::_1)\
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_filtered", 10);
}


void RspclFilterComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // define a new container for the data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZRGB>);

  // // PassThrough Filter
  // pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("x");  // x axis
  // pass.setFilterLimits(-0.3, 0.3);
  // pass.setFilterLimitsNegative(false);   // Keep points outside this range
  // pass.filter(*cloud_filtered_x);

  // // PassThrough Filter for y axis
  // pass.setInputCloud(cloud_filtered_x);
  // pass.setFilterFieldName("y");  // y axis
  // pass.setFilterLimits(-0.1, 1.0);
  // pass.setFilterLimitsNegative(false);   // Keep points outside this range
  // pass.filter(*cloud_filtered_y);

  // // PassThrough Filter for z axis
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");  // z axis
  pass.setFilterLimits(-1.3, 0.4);
  pass.setFilterLimitsNegative(false);   // Keep points outside this range
  pass.filter(*cloud_filtered);

  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(cloud_filtered);
  crop.setMin(Eigen::Vector4f(-0.3, -0.9, -1.4, 1.0));
  crop.setMax(Eigen::Vector4f(0.3, 0.1, 1.0, 1.0));
  crop.setNegative(true); // Keep points outside the box
  crop.filter(*cloud_filtered);

  // // Approximate Voxel Grid
  // pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avg;
  // avg.setInputCloud(cloud);
  // avg.setLeafSize(0.2f, 0.2f, 0.2f);
  // // avg.setDownsampleAllData(true);
  // avg.filter(*cloud_filtered);

  // Voxel Grid: pattern 1
  // pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  // voxelGrid.setInputCloud(cloud_filtered);
  // leaf_size_ = 0.1;
  // // set the leaf size (x, y, z)
  // voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // // apply the filter to dereferenced cloudVoxel
  // voxelGrid.filter(*cloud_filtered);

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud (cloud_filtered);
  // sor.setMeanK (20); //设置考虑查询点临近点数
  // sor.setStddevMulThresh (10);//设置判断是否为离群点的阀值
  // sor.filter (*cloud_filtered);
  // sor.setNegative (false);
  // sor.filter(*cloud_filtered);

  // // Voxel Grid: pattern 2
  // pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
  // pcl::toPCLPointCloud2(*cloud, *cloud_blob);
  // pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  // vg.setInputCloud(cloud_blob);
  // leaf_size_ = 0.1;
  // vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // vg.filter(*cloud_filtered_blob);
  // pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  // // Statistical Outlier Removal
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud(cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(0.1);
  // sor.setNegative(false);
  // sor.filter (*cloud_filtered);

  // // Radius Outlier Removal
  // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // outrem.setInputCloud(cloud);
  // outrem.setRadiusSearch(0.1);
  // outrem.setMinNeighborsInRadius(2);
  // outrem.setKeepOrganized(true);
  // outrem.filter(*cloud_filtered);

  // // Conditional Removal
  // pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, 0.0)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 3.0)));
  // pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud);
  // // condrem.setKeepOrganized(true);
  // condrem.filter(*cloud_filtered);
  // // vector<int> Idx;
  // // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, Idx);


  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  // Set the correct timestamp
  sensor_msg.header.stamp = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Publishing with timestamp: %u.%u", sensor_msg.header.stamp.sec, sensor_msg.header.stamp.nanosec);
  publisher_->publish(sensor_msg);
}