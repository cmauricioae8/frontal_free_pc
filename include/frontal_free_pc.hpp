#ifndef FRONTAL_FREE_PC_HPP
#define FRONTAL_FREE_PC_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <functional>

using namespace std::chrono_literals;

namespace free_pc_ns {

class FrontalFreePC : public rclcpp::Node
{
  public:
    explicit FrontalFreePC(const rclcpp::NodeOptions &options);

    ~FrontalFreePC() override;

    rcl_interfaces::msg::SetParametersResult params_CB(const std::vector<rclcpp::Parameter>& params);
    void publisher_checker();
    void pointcloud2_CB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frontal_free_pc_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    sensor_msgs::msg::PointCloud2 cloud_msg_;
    std::string sensor_frame_, target_frame_;
    std_msgs::msg::Float32 frontal_pc_msg_;
    visualization_msgs::msg::Marker marker_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    double cb_time_, cb_latency_, cb_hz_, last_cloud_time_;
    double max_depth_x_, robot_width_, min_height_, max_height_;
    bool debug_mode_ = false;
    std::string debug_point_topic_;
    int reduce_resolution_;
    double min_val_pc_ant_ = 0, filt_gain_, bias_ = 0.01; // First order low-pass filter (0,1)
    OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};

} // namespace frontal_pc_ns

#endif //FRONTAL_FREE_PC_HPP