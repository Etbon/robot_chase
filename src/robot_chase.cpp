#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {
  public:
    RobotChase()
        : Node("robot_chase_node") {

        // Declaring parameters
        kp_distance =  this->declare_parameter("kp_distance", 0.5);
        kp_yaw = this->declare_parameter("kp_yaw", 1.0);

        // Initialize buffer 
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        
        // Initialize listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

        // Timer 
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&RobotChase::chaseLogic, this)
        );

    } 

  private:
    // Define member variables 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double kp_distance;
    double kp_yaw;


    // Control loop 
    void chaseLogic() {
        // 1. Try to get data from the world TF
        geometry_msgs::msg::TransformStamped tf_rick_to_morty;
        
        try { 
            // Try to get the most recent transforms from Rick to Morty
            tf_rick_to_morty = tf_buffer_->lookupTransform("rick/base_link", "morty/base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        
        // 2. Extract meaning from the data  
        // 2.1 Get the X and Y offset from the transform
        double dx = tf_rick_to_morty.transform.translation.x;
        double dy = tf_rick_to_morty.transform.translation.y;
        
        // 2.2 Compute the error distance & error yaw
        double error_distance = std::hypot(dx, dy);
        double error_yaw = std::atan2(dy, dx);
        
        // 2.3 Set proportional control 
        double linear_velocity = kp_distance * error_distance;
        double angular_velocity = kp_yaw * error_yaw;

        // 3. Stop pushing if close 
        if (error_distance < 0.1) {
            linear_velocity = 0.0;
            angular_velocity = 0.0;
        }

        // 4. Creat and publish Twist message 
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
 
        publisher_->publish(cmd_vel);
    }   
};

int main(int argc, char**argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotChase>());
    rclcpp::shutdown();
    return 0;

}