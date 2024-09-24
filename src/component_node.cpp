/* 
@description:
This component is the equivalent to the 'frontal_free_pc' node.

Both the header file and this implementation file can be joined into one file.
This is optional.

@author: C. Mauricio Arteaga-Escamilla
*/

#include "frontal_free_pc.hpp"
#include <rclcpp/rclcpp.hpp>

std::string node_name = "frontal_free_pc";
using namespace std::chrono_literals;

namespace free_pc_ns
{

  FrontalFreePC::FrontalFreePC(const rclcpp::NodeOptions &options) : Node(node_name, options)
  {
    auto sensor_QoS = rclcpp::SensorDataQoS();
    auto default_QoS = rclcpp::SystemDefaultsQoS();

    frontal_free_pc_pub_ = this->create_publisher<std_msgs::msg::Float32>("frontal_free_pc", default_QoS);

    debug_point_topic_ = node_name+"/point";
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(debug_point_topic_, 1);

    // Checking if namespace provided
    std::string ns = this->get_namespace();
    if(ns.size() > 1){
      RCLCPP_INFO(this->get_logger(), "The provided namespace is: %s", ns.c_str());
      debug_point_topic_ = ns+"/"+debug_point_topic_;
    }
      
    
    pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud", sensor_QoS,
                        std::bind(&FrontalFreePC::pointcloud2_CB, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(1000ms, std::bind(&FrontalFreePC::publisher_checker, this));

    // ROS params
    max_depth_x_ = this->declare_parameter<double>("max_depth_x", 4);
    robot_width_ = this->declare_parameter<double>("robot_width", 0.7);
    min_height_ = this->declare_parameter<double>("min_height", 0.005);
    max_height_ = this->declare_parameter<double>("max_height", 1.7);
    
    target_frame_ = this->declare_parameter("target_frame", "base_footprint");
    // target_frame_ = this->get_parameter("target_frame").as_string();

    debug_mode_ = this->declare_parameter<bool>("debug_mode", true);
    reduce_resolution_ = this->declare_parameter<int>("reduce_resolution", 1);
    filt_gain_ = this->declare_parameter<double>("filter_gain", 0.1);

    RCLCPP_INFO(this->get_logger(), "Initial ROS params");
    RCLCPP_INFO(this->get_logger(), "max_depth_x: %.3f", max_depth_x_);
    RCLCPP_INFO(this->get_logger(), "robot_width: %.3f (fixed)", robot_width_);
    RCLCPP_INFO(this->get_logger(), "min_height: %.3f (fixed)", min_height_);
    RCLCPP_INFO(this->get_logger(), "max_height: %.3f (fixed)", max_height_);
    RCLCPP_INFO(this->get_logger(), "target_frame: %s (fixed)", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "debug_mode: %d", debug_mode_);
    RCLCPP_INFO(this->get_logger(), "reduce_resolution: %d", reduce_resolution_);
    RCLCPP_INFO(this->get_logger(), "filter_gain: %.2f", filt_gain_);

    if(debug_mode_){
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing point for debugging on '" 
          << debug_point_topic_ << "' topic");
    } 
    else RCLCPP_WARN(this->get_logger(), "Not publishing point for debugging. Topic removed");


    // Marker definition
    marker_msg_.header.frame_id = target_frame_;
    marker_msg_.scale.x = 0.1;
    marker_msg_.scale.y = 0.1;
    marker_msg_.scale.z = 0.1;
    marker_msg_.id = 1;
    marker_msg_.color.r = 1.0;
    marker_msg_.color.a = 0.9;
    marker_msg_.type = marker_msg_.SPHERE; //CUBE
    marker_msg_.ns = "m1";
    marker_msg_.lifetime.sec = 1;


    params_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FrontalFreePC::params_CB, this, std::placeholders::_1));

    last_cloud_time_ = now().seconds();
    RCLCPP_INFO(this->get_logger(), "Component initialized");
  }

  FrontalFreePC::~FrontalFreePC(){ RCLCPP_INFO(this->get_logger(), "Component finished"); }

  rcl_interfaces::msg::SetParametersResult FrontalFreePC::params_CB(const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;

    // Since the most of params are fixed, only variable params are validated
    result.successful = false;
    result.reason = "This ROS param cannot be modified at runtime";

    for(const auto &param: params){
      if (param.get_name() == "max_depth_x"){
        double min_val = 0.5;
        std::stringstream min_val_str;    min_val_str << min_val;
        if (param.as_double() >= min_val){
          max_depth_x_ = param.as_double();
          RCLCPP_INFO(this->get_logger(), "max_depth_x set to: %.3f m", max_depth_x_);
          result.successful = true;
          result.reason = "OK. ROS param updated";
        }else{
          result.reason = "The max_depth_x minimum value is: " + min_val_str.str();
        }
      }
      if (param.get_name() == "debug_mode"){
        debug_mode_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "debug_mode set to: %d", debug_mode_);
        result.successful = true;
        if(debug_mode_){
          RCLCPP_INFO_STREAM(this->get_logger(), "Publishing point for debugging on '" 
            << debug_point_topic_ << "' topic");
          result.reason = "OK. ROS param updated, Publishing point for debugging";
        }else{
          RCLCPP_INFO(this->get_logger(), "Not publishing point for debugging. Topic removed");
          result.reason = "OK. ROS param updated, NOT publishing point for debugging. Topic removed";
          marker_pub_.reset();
        }
      }
      if (param.get_name() == "reduce_resolution"){
        int min_val = 1;
        std::stringstream min_val_str;    min_val_str << min_val;
        if (param.as_int() >= min_val){
          reduce_resolution_ = param.as_int();
          RCLCPP_INFO(this->get_logger(), "reduce_resolution set to: %d", reduce_resolution_);
          result.successful = true;
          result.reason = "OK. ROS param updated";
        }else{
          result.reason = "The reduce_resolution minimum value is: " + min_val_str.str();
        }
      }
      if (param.get_name() == "filter_gain"){
        if (param.as_double() > 0 && param.as_double() < 1){
          filt_gain_ = param.as_double();
          RCLCPP_INFO(this->get_logger(), "filter_gain set to: %.3f", filt_gain_);
          result.successful = true;
          result.reason = "OK. ROS param updated";
        }else{
          result.reason = "The filter_gain must be in (0,1)";
        }
      }
    }
    return result; 
  }

  void FrontalFreePC::publisher_checker()
  {
    double current_time = now().seconds();
    if( current_time - last_cloud_time_ > 5){
      RCLCPP_WARN_ONCE(this->get_logger(), "pointcloud topic not publishing");
      RCLCPP_INFO_ONCE(this->get_logger(), "Check if pointcloud topic remapping is correct");
    }
  }

  void FrontalFreePC::pointcloud2_CB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    cb_time_ = now().seconds();
    cb_hz_ = 1/(cb_time_ - last_cloud_time_);
    last_cloud_time_ = cb_time_;

    //Create a transform stamped object
    geometry_msgs::msg::TransformStamped stfT;
    sensor_frame_ = cloud_msg->header.frame_id;

    // Transform cloud if necessary
    if (sensor_frame_ != target_frame_ && !target_frame_.empty()) {
      try{
        stfT = tf_buffer_->lookupTransform( target_frame_, sensor_frame_, tf2::TimePointZero);
      }catch (const tf2::TransformException & ex) {
        RCLCPP_WARN( this->get_logger(), "Could not transform %s to %s: %s",
            target_frame_.c_str(), sensor_frame_.c_str(), ex.what());
        RCLCPP_INFO(this->get_logger(), "Please check the TF tree to prevent any possible problem");
        return;
      } 
    }

    //Transformation between coordinates frames
    double stfX, stfY, stfZ; //translation from sensor to target frame
    stfX = stfT.transform.translation.x;
    stfY = stfT.transform.translation.y;
    stfZ = stfT.transform.translation.z;

    // Quaternion
    tf2::Quaternion q( stfT.transform.rotation.x, stfT.transform.rotation.y,
            stfT.transform.rotation.z, stfT.transform.rotation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q); 
    
    tf2::Vector3 row0, row1, row2; //Rows of the rotation matrix  
    // Extract the rows of the rotation matrix, row.{x,y,z}
    row0 = m.getRow(0);
    row1 = m.getRow(1);
    row2 = m.getRow(2);

    float x_t, y_t, z_t; //cartesian coordinates with respect to target frame
    float x_s_min = max_depth_x_, y_s_min, z_s_min; // A large value

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
        iter_y(*cloud_msg, "y"),iter_z(*cloud_msg, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if(reduce_resolution_ > 1){
        iter_x = iter_x + reduce_resolution_;
        iter_y = iter_y + reduce_resolution_;
        iter_z = iter_z + reduce_resolution_;
        if( &*iter_x > &*iter_x.end() ) break;
      }

      // iter_XYZ is given with respect to sensor frame 
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;
      if (std::isinf(*iter_x) || std::isinf(*iter_y) || std::isinf(*iter_z)) continue;
  
      // Convert from sensor frame to target frame
      x_t = row0.getX()*(*iter_x) + row0.getY()*(*iter_y) + row0.getZ()*(*iter_z) + stfX;
      y_t = row1.getX()*(*iter_x) + row1.getY()*(*iter_y) + row1.getZ()*(*iter_z) + stfY;
      z_t = row2.getX()*(*iter_x) + row2.getY()*(*iter_y) + row2.getZ()*(*iter_z) + stfZ;

      // Size Validations
      if(abs(y_t) > robot_width_/2) continue;
      if(x_t > max_depth_x_) continue;
      if(z_t > max_height_ || z_t < min_height_) continue;

      if (x_t < x_s_min){
        x_s_min = x_t; y_s_min = y_t; z_s_min = z_t;
      }
    }
    
    // filtered valor_minx with bias
    double valor_minx_filt = filt_gain_*min_val_pc_ant_ + (1-filt_gain_)*x_s_min -bias_; 
    min_val_pc_ant_ = valor_minx_filt;

    frontal_pc_msg_.data = valor_minx_filt;
    frontal_free_pc_pub_->publish(frontal_pc_msg_);

    cb_latency_ = now().seconds() - cb_time_;

    if(debug_mode_){
      // Marker for the nearest point along X axis
      marker_msg_.header.stamp.sec = now().seconds();
      marker_msg_.pose.position.x = x_s_min;
      marker_msg_.pose.position.y = y_s_min;
      marker_msg_.pose.position.z = z_s_min;
      //marker_msg_.action = 3; // deletes all objects (or those with the given ns if any)
      marker_pub_->publish(marker_msg_);

      RCLCPP_INFO(this->get_logger(), "x_min: %.3f,\ty_min: %.3f,\tz_min: %.3f", x_s_min,y_s_min,z_s_min);
      RCLCPP_INFO(this->get_logger(), "lat: %.3f,\tcb_hz: %.2f", cb_latency_, cb_hz_);
    }
  }

} // namespace frontal_pc_ns

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(free_pc_ns::FrontalFreePC)