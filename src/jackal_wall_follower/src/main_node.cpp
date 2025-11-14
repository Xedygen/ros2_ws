#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

class WallFollowNode : public rclcpp::Node
{
public:
    WallFollowNode() : Node("wall_follow_node"),
        desired_distance_(1.5),
        max_linear_(1.0),
        max_angular_(2.0),
        panic_distance_(2.0),
        min_lidar_range_(0.9),
        side_threshold_(0.2),
        min_safe_distance_(1.1),
        wall_lost_threshold_(4.0),
        turning_(false),
        turn_progress_(0.0)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollowNode::scan_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double right_sum = 0.0, right_count = 0.0;
        double front_sum = 0.0, front_count = 0.0;
        
        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;
        
        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double r = msg->ranges[i];
            if(std::isinf(r) || r < min_lidar_range_) continue;
            
            double angle = angle_min + i * angle_inc;
            
            if(angle > -M_PI/2 && angle < -M_PI/6) { 
                right_sum += r; 
                right_count++; 
            }

            if(angle > -M_PI/12 && angle < M_PI/12) { 
                front_sum += r; 
                front_count++; 
            }

        }
        
        double right_dist = (right_count > 0) ? right_sum/right_count : 5.0;
        double front_dist = (front_count > 0) ? front_sum/front_count : 5.0;
        
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        
        if(turning_)
        {
            handleTurning(cmd, right_dist, front_dist);
        }
        else
        {

            if(front_dist < panic_distance_)
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected! Turning left. Front: %.2fm", front_dist);
                initiateTurn("left");
                cmd.angular.z = max_angular_;
                cmd.linear.x = 0.0;
                rclcpp::Rate loop_rate(1.0);
            }

            else if(right_dist > wall_lost_threshold_)
            {
                RCLCPP_INFO(this->get_logger(), "Wall lost on right! Turning right. Right: %.2fm", right_dist);
                initiateTurn("right");
                cmd.angular.z = -max_angular_;
                rclcpp::Rate loop_rate(1.0);
                cmd.linear.x = 0.0;
            }

            else
            {
                followWall(cmd, right_dist, front_dist);
            }
        }
        
        cmd_pub_->publish(cmd);
    }
    
    void initiateTurn(const std::string& direction)
    {
        turning_ = true;
        turn_direction_ = direction;
        turn_progress_ = 0.0;
        turn_start_time_ = this->now();
    }
    
    void handleTurning(geometry_msgs::msg::Twist& cmd, double right_dist, double front_dist)
    {

        auto elapsed = this->now() - turn_start_time_;
        turn_progress_ = elapsed.seconds() * max_angular_;
        
        if(turn_direction_ == "left")
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = max_angular_;
            rclcpp::Rate loop_rate(1.0);
        }
        else
        {
            cmd.linear.x = max_linear_;
            cmd.angular.z = -max_angular_ * 0.4;
            rclcpp::Rate loop_rate(0.2);
        }
        
        if(turn_progress_ >= M_PI_2)
        {
            turning_ = false;
            turn_progress_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "90 degree turn complete!");
        }
    }
    
    void followWall(geometry_msgs::msg::Twist& cmd, double right_dist, double front_dist)
    {
        double error = right_dist - desired_distance_;
        double kp_angular = 2.0; 
        
        cmd.angular.z = -kp_angular * error;
        
        cmd.angular.z = std::max(-max_angular_, std::min(max_angular_, cmd.angular.z));
        
        if(front_dist < panic_distance_ * 0.6)
        {
            cmd.linear.x = max_linear_ * 0.2;
        }
        else if(front_dist < panic_distance_) 
        {
            cmd.linear.x = max_linear_ * 0.5;
        }
        else if(right_dist < min_safe_distance_)
        {
            cmd.linear.x = max_linear_ * 0.4;
        }
        else if(front_dist < desired_distance_ * 2.5) 
        {
            cmd.linear.x = max_linear_ * 0.7;
        }
        else
        {
            double speed_factor = 1.0 - std::abs(cmd.angular.z) / max_angular_ * 0.3;
            cmd.linear.x = max_linear_ * speed_factor;
        }
        
        if(right_dist < min_safe_distance_)
        {
            cmd.angular.z = std::max(cmd.angular.z, 0.8); 
            cmd.linear.x = std::min(cmd.linear.x, max_linear_ * 0.1);
            RCLCPP_WARN(this->get_logger(), "Too close to wall! Right: %.2f", right_dist);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    double desired_distance_;
    double max_linear_;
    double max_angular_;
    double panic_distance_;
    double min_lidar_range_;
    double side_threshold_;
    double min_safe_distance_;
    double wall_lost_threshold_;
    
    bool turning_;
    std::string turn_direction_;
    rclcpp::Time turn_start_time_;
    double turn_progress_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowNode>());
    rclcpp::shutdown();
    return 0;
}