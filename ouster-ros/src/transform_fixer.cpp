#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

class TransformRepublisher : public rclcpp::Node
{
public:
    TransformRepublisher() : Node("transform_republisher")
    {
        // Declare parameters with default values
        this->declare_parameter("fcu_frame", "uav30/fcu");
        this->declare_parameter("old_os_sensor_frame", "os_sensor");
        this->declare_parameter("new_os_sensor_frame", "uav30/os_sensor");
        
        // Get parameter values
        fcu_frame_ = this->get_parameter("fcu_frame").as_string();
        old_os_sensor_frame_ = this->get_parameter("old_os_sensor_frame").as_string();
        new_os_sensor_frame_ = this->get_parameter("new_os_sensor_frame").as_string();
        
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer to periodically check for transform and republish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&TransformRepublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Transform republisher node started");
        RCLCPP_INFO(this->get_logger(), "Looking for transform: %s -> %s", 
                    fcu_frame_.c_str(), old_os_sensor_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Will republish as: %s -> %s", 
                    fcu_frame_.c_str(), new_os_sensor_frame_.c_str());
    }

private:
    void timer_callback()
    {
        try
        {
            // Look up the transform using parameterized frame names
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                fcu_frame_, old_os_sensor_frame_, tf2::TimePointZero);
            
            // Create new transform with modified frame names
            geometry_msgs::msg::TransformStamped new_transform;
            new_transform.header.stamp = this->get_clock()->now();
            new_transform.header.frame_id = fcu_frame_;
            new_transform.child_frame_id = new_os_sensor_frame_;
            
            // Copy the transform data (translation and rotation)
            new_transform.transform = transform_stamped.transform;
            
            // Broadcast the new transform
            tf_broadcaster_->sendTransform(new_transform);
            
            // Log success (only once to avoid spam)
            if (!transform_found_)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully found and republishing transform: %s -> %s", 
                    fcu_frame_.c_str(), new_os_sensor_frame_.c_str());
                transform_found_ = true;
            }
        }
        catch (const tf2::TransformException& ex)
        {
            // Only log the error occasionally to avoid spam
            if (error_count_++ % 50 == 0)  // Log every 5 seconds at 10Hz
            {
                RCLCPP_WARN(this->get_logger(), 
                    "Could not transform %s to %s: %s", 
                    fcu_frame_.c_str(), old_os_sensor_frame_.c_str(), ex.what());
            }
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool transform_found_ = false;
    int error_count_ = 0;
    
    // Parameter variables
    std::string fcu_frame_;
    std::string old_os_sensor_frame_;
    std::string new_os_sensor_frame_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformRepublisher>());
    rclcpp::shutdown();
    return 0;
}