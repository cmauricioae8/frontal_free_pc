/* 
@description: 

@author: C. Mauricio Arteaga-Escamilla


To test this component without using the 'component_launch.py'

In one terminal:
$ ros2 run rclcpp_components component_container   #Create a component container

In another terminal:
$ ros2 component load /ComponentManager frontal_free_pc free_pc_ns::FrontalFreePC   # To load (start) the component
$ ros2 component unload /ComponentManager <component_ID>   # To unload (finish) the component


ToDo:
+ Split into include folder
+ 
*/

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
#include <string>
#include <vector>
#include <chrono>
#include <thread>

std::string node_name = "frontal_free_pc";
using namespace std::chrono_literals;

namespace free_pc_ns {

class FrontalFreePC : public rclcpp::Node
{
    public:
    FrontalFreePC(const rclcpp::NodeOptions &options) : Node(node_name, options)
    {
        auto sensor_QoS = rclcpp::SensorDataQoS();
        auto default_QoS = rclcpp::SystemDefaultsQoS();

        frontal_free_pc_pub_ = this->create_publisher<std_msgs::msg::Float32>("frontal_free_pc", default_QoS);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontal_free_pc/point", 1);
        
        pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud", sensor_QoS,
                            std::bind(&FrontalFreePC::pointcloud2_CB, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(1000ms, std::bind(&FrontalFreePC::timer_function, this));

        // ROS params
        this->declare_parameter<double>("max_depth_x", 4);
        this->declare_parameter<double>("robot_width", 0.7);
        this->declare_parameter<double>("min_height", 0.005);
        this->declare_parameter<double>("max_height", 1.7);
        this->declare_parameter("target_frame", "base_footprint");
        this->declare_parameter<bool>("debug_mode", true);
        this->declare_parameter<int>("reduce_resolution", 1);
        this->declare_parameter<double>("alpha", 0.1);

        get_parameter("max_depth_x", max_depth_x_);
        get_parameter("robot_width", robot_width_);
        get_parameter("min_height", min_height_);
        get_parameter("max_height", max_height_);
        target_frame_ = this->get_parameter("target_frame").as_string();
        get_parameter("debug_mode", debug_mode_);
        get_parameter("reduce_resolution", reduce_resolution_);
        get_parameter("alpha", alpha_);

        RCLCPP_INFO(this->get_logger(), "max_depth_x: %.3f", max_depth_x_);
        RCLCPP_INFO(this->get_logger(), "robot_width: %.3f", robot_width_);
        RCLCPP_INFO(this->get_logger(), "min_height: %.3f", min_height_);
        RCLCPP_INFO(this->get_logger(), "max_height: %.3f", max_height_);
        RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "debug_mode: %d", debug_mode_);
        RCLCPP_INFO(this->get_logger(), "reduce_resolution: %d", reduce_resolution_);
        RCLCPP_INFO(this->get_logger(), "alpha: %.2f", alpha_);

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

        last_cloud_time_ = now().seconds();
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

    ~FrontalFreePC(){ RCLCPP_INFO(this->get_logger(), "Component finished"); }

    void timer_function()
    {
        double current_time = now().seconds();
        if( current_time - last_cloud_time_ > 5){
            RCLCPP_WARN_ONCE(this->get_logger(), "pointcloud topic is not publishing");
            RCLCPP_INFO_ONCE(this->get_logger(), "Check if pointcloud topic remapping is correct");
        }
    }

    void pointcloud2_CB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
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
        double valor_minx_filt = alpha_*min_val_pc_ant_ + (1-alpha_)*x_s_min -bias_; 
        min_val_pc_ant_ = valor_minx_filt;

        frontal_pc_msg_.data = valor_minx_filt;
        frontal_free_pc_pub_->publish(frontal_pc_msg_);

        freq_div_++;
        if(freq_div_ == 30){
            freq_div_ = 0;
            get_parameter("debug_mode", debug_mode_);
            get_parameter("reduce_resolution", reduce_resolution_);
            get_parameter("alpha", alpha_);

            if(reduce_resolution_ < 1) reduce_resolution_ = 1;
        }

        cb_latency_ = now().seconds() - cb_time_;

        if(debug_mode_){
            // Marker for the nearest point along X axis
            marker_msg_.header.stamp.sec = now().seconds();
            marker_msg_.pose.position.x = x_s_min;
            marker_msg_.pose.position.y = y_s_min;
            marker_msg_.pose.position.z = z_s_min;
            //marker_msg_.action = 3; // deletes all objects (or those with the given ns if any)
            marker_pub_->publish(marker_msg_);

            RCLCPP_INFO( this->get_logger(), "x_min: %.3f,\ty_min: %.3f,\tz_min: %.3f", x_s_min,y_s_min,z_s_min);
            RCLCPP_INFO( this->get_logger(), "lat: %.3f,\tcb_hz: %.2f", cb_latency_, cb_hz_);
        }        
    }

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

        double cb_time_, cb_latency_, cb_hz_;
        double max_depth_x_, robot_width_, min_height_, max_height_;
        // rclcpp::CallbackGroup::SharedPtr cb_group_;
        bool debug_mode_ = false, debug_mode_prev_ = false;
        int reduce_resolution_, freq_div_ = 0;
        double last_cloud_time_;

        // First order low-pass filter (0 < alpha < 1)
        double min_val_pc_ant_ = 0, alpha_, bias_ = 0.01;
};

} // namespace frontal_pc_ns

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(free_pc_ns::FrontalFreePC)