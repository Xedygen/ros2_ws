#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>

using std::placeholders::_1;

// --- Configuration Constants ---
const double LINEAR_SPEED = 0.5;   // m/s - Forward speed when path is clear
const double ANGULAR_SPEED = 0.8;  // rad/s - Speed for rotational movement
const double KP_STEER = 1.0;       // Proportional gain for steering
const double TARGET_WALL_DISTANCE = 0.9; // m - Target distance to maintain from the right wall (half of 1.8m path width)
const double STOP_OBSTACLE_DISTANCE = 0.9; // m - Distance considered "blocked" directly ahead

// Wall detection threshold (If distance is > this, there's an opening/no wall)
const double MAX_WALL_DISTANCE = 1.6;

// Fixed time calculation for 90-degree turn (PI/2 radians at 0.8 rad/s)
// This is used for dead ends (Left Turns) but is avoided for Right turns.
const double TURN_DURATION_90_DEG = M_PI / 2.0 / ANGULAR_SPEED; 

// Define states for the robot's behavior
enum class State {
    FORWARD_FOLLOW_RIGHT,
    COMMITTING_TURN
};

class RightWallFollower : public rclcpp::Node
{
public:
    RightWallFollower() : Node("right_wall_follower")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&RightWallFollower::scanCallback, this, _1));
            
        current_state_ = State::FORWARD_FOLLOW_RIGHT;
        turn_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "RightWallFollower Node Initialized. Following Right Wall.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    State current_state_;
    rclcpp::Time turn_start_time_;
    double turn_duration_ = 0.0;
    double turn_direction_ = 0.0; // 1.0 for left turn, -1.0 for right turn

    /**
     * @brief Extracts the minimum range distance for a given angular segment.
     * @param data The LaserScan message.
     * @param angle_deg The central angle of the segment (in degrees).
     * @param width_deg The width of the segment (total angle) to check over.
     * @return The minimum valid distance in meters in that segment.
     */
    double getMinSegmentDistance(const sensor_msgs::msg::LaserScan::SharedPtr data, 
                                 double angle_deg, double width_deg = 10.0)
    {
        double angle_rad = angle_deg * M_PI / 180.0;
        double min_angle_rad = data->angle_min;
        double angle_increment = data->angle_increment;
        size_t ranges_size = data->ranges.size();

        double start_angle = angle_rad - (width_deg / 2.0) * M_PI / 180.0;
        double end_angle = angle_rad + (width_deg / 2.0) * M_PI / 180.0;

        size_t start_index = static_cast<size_t>(std::floor((start_angle - min_angle_rad) / angle_increment));
        size_t end_index = static_cast<size_t>(std::ceil((end_angle - min_angle_rad) / angle_increment));

        // Clamp indices to valid range
        start_index = std::max(0UL, start_index);
        end_index = std::min(ranges_size, end_index);

        double min_range = std::numeric_limits<double>::infinity();

        for (size_t i = start_index; i < end_index; ++i)
        {
            double range = data->ranges[i];
            if (std::isnormal(range) && range >= data->range_min)
            {
                min_range = std::min(min_range, range);
            }
        }
        return min_range;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. Get distance readings
        double D_front = getMinSegmentDistance(msg, 0.0, 10.0);    // Check directly ahead
        double D_right = getMinSegmentDistance(msg, -90.0, 10.0);  // Check right side (for following)
        double D_front_right = getMinSegmentDistance(msg, -45.0, 5.0); // Check diagonal right (for corner anticipation)
        
        auto twist = geometry_msgs::msg::Twist();

        switch (current_state_)
        {
            case State::FORWARD_FOLLOW_RIGHT:
            {
                // --- Decision Logic Priority (Right-Hand Rule) ---

                // RULE 3: Open Corridor / Turn Right (High Priority)
                // If the diagonal right path is wide open, commit to a RIGHT turn.
                if (D_front_right > MAX_WALL_DISTANCE)
                {
                    current_state_ = State::COMMITTING_TURN;
                    turn_start_time_ = this->now();
                    turn_duration_ = TURN_DURATION_90_DEG;
                    turn_direction_ = -1.0; // Right turn (Negative angular velocity)
                    RCLCPP_WARN(this->get_logger(), "Right Opening (%.2fm). Committing 90 deg RIGHT Turn.", D_front_right);
                }
                // RULE 2: Front Blocked / Must Turn Left (Dead End)
                // If the front is blocked and D_front_right is NOT open (i.e., we hit a wall/corner)
                else if (D_front < STOP_OBSTACLE_DISTANCE)
                {
                    current_state_ = State::COMMITTING_TURN;
                    turn_start_time_ = this->now();
                    turn_duration_ = TURN_DURATION_90_DEG;
                    turn_direction_ = 1.0; // Left turn (Positive angular velocity)
                    RCLCPP_WARN(this->get_logger(), "Front Blocked (%.2fm). Committing 90 deg LEFT Turn.", D_front);
                }
                // RULE 1: Default to Right Wall Following
                else
                {
                    // Calculate P-Control for alignment
                    double wall_following_error = TARGET_WALL_DISTANCE - D_right;
                    double angular_vel = -KP_STEER * wall_following_error; 
                    
                    // Cap rotation speed
                    angular_vel = std::min(std::max(angular_vel, -ANGULAR_SPEED), ANGULAR_SPEED);

                    twist.linear.x = LINEAR_SPEED;
                    twist.angular.z = angular_vel;
                    RCLCPP_INFO(this->get_logger(), "Following Right Wall (D_R: %.2fm). Angular: %.2f", D_right, angular_vel);
                }
                break;
            }

            case State::COMMITTING_TURN:
            {
                // Execute fixed-time turn
                rclcpp::Duration elapsed_time = this->now() - turn_start_time_;
                
                if (elapsed_time.seconds() < turn_duration_)
                {
                    // Continue turning for the calculated duration
                    twist.linear.x = 0.0; // Rotate in place
                    twist.angular.z = turn_direction_ * ANGULAR_SPEED; 
                    RCLCPP_INFO(this->get_logger(), "Turning (Time Left: %.2f / %.2f s)", turn_duration_ - elapsed_time.seconds(), turn_duration_);
                }
                else
                {
                    // Turn duration complete, switch back to forward following
                    current_state_ = State::FORWARD_FOLLOW_RIGHT;
                    RCLCPP_INFO(this->get_logger(), "90-degree Turn Complete. Resuming Right Wall Follow.");
                    
                    // Publish stop command immediately to zero out velocity
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                }
                break;
            }
        }

        // 4. Publish Command
        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RightWallFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}