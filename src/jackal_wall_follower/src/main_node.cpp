#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

using std::placeholders::_1;

// --- Configuration Constants ---
// These should be tuned based on your robot's speed and maze cell size (1.8m)
const double LINEAR_SPEED = 1;   // m/s
const double KP_STEER = 0.8;       // Proportional gain for steering
const double MAX_ANGULAR_VEL = 0.8; // rad/s (Also used as dedicated turn speed)

// Thresholds for Wall Detection
const double MAX_CORRIDOR_WIDTH = 1.8; 
const double MIN_CORRIDOR_WIDTH = 1.8;
const double MAX_RANGE_REPLACEMENT = 50.0; // Value to use if max range/inf is detected

class MidPathFollower : public rclcpp::Node
{
public:
    MidPathFollower() : Node("mid_path_follower")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MidPathFollower::scanCallback, this, _1));
        RCLCPP_INFO(this->get_logger(), "MidPathFollower Node Initialized.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    /**
     * @brief Extracts the average range distance for a given angular segment.
     * @param data The LaserScan message.
     * @param angle_deg The central angle of the segment (in degrees).
     * @param width_deg The width of the segment (total angle) to average over.
     * @return The filtered average distance in meters.
     */
    double getSegmentDistance(const sensor_msgs::msg::LaserScan::SharedPtr data, 
                              double angle_deg, double width_deg = 5.0)
    {
        double angle_rad = angle_deg * M_PI / 180.0;
        double min_angle_rad = data->angle_min;
        double angle_increment = data->angle_increment;
        size_t ranges_size = data->ranges.size();

        // Calculate start and end index
        double start_angle = angle_rad - (width_deg / 2.0) * M_PI / 180.0;
        double end_angle = angle_rad + (width_deg / 2.0) * M_PI / 180.0;

        size_t start_index = static_cast<size_t>(std::floor((start_angle - min_angle_rad) / angle_increment));
        size_t end_index = static_cast<size_t>(std::ceil((end_angle - min_angle_rad) / angle_increment));

        // Clamp indices
        start_index = std::max(0UL, start_index);
        end_index = std::min(ranges_size, end_index);

        std::vector<double> valid_ranges;
        for (size_t i = start_index; i < end_index; ++i)
        {
            double range = data->ranges[i];
            // Filter out invalid readings (NaN, Inf, or below min_range)
            if (std::isnormal(range) && range >= data->range_min && range <= data->range_max)
            {
                valid_ranges.push_back(range);
            }
        }

        if (valid_ranges.empty())
        {
            // If no valid readings, return a large distance to indicate open space
            return MAX_RANGE_REPLACEMENT;
        }

        // Calculate average
        double sum = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0);
        return sum / valid_ranges.size();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. Get filtered distance readings for key directions
        
        // Frontal scan (0 degrees)
        double D_front = getSegmentDistance(msg, 0.0);
        // Right side (90 degrees clockwise = -90 degrees)
        double D_right = getSegmentDistance(msg, -90.0);
        // Left side (90 degrees counter-clockwise = +90 degrees)
        double D_left = getSegmentDistance(msg, 90.0);

        auto twist = geometry_msgs::msg::Twist();
        double angular_vel = 0.0;

        // 2. OBSTACLE/BLOCKED FRONT CHECK
        if (D_front < MIN_CORRIDOR_WIDTH)
        {
            // Stop forward motion immediately
            twist.linear.x = 0.0;
            
            // Decide turn direction: rotate towards the side with max clearance
            if (D_left > D_right)
            {
                // Turn Left (positive angular velocity)
                angular_vel = MAX_ANGULAR_VEL;
                RCLCPP_WARN(this->get_logger(), "Front Blocked (%.2fm). Rotating Left (Clearance L: %.2fm).", D_front, D_left);
            }
            else
            {
                // Turn Right (negative angular velocity)
                angular_vel = -MAX_ANGULAR_VEL;
                RCLCPP_WARN(this->get_logger(), "Front Blocked (%.2fm). Rotating Right (Clearance R: %.2fm).", D_front, D_right);
            }
        }
        else 
        {
            // 3. CENTERING LOGIC (Path is clear ahead)
            twist.linear.x = LINEAR_SPEED;
            
            // A corridor is detected if both left and right see a wall within the maximum expected width
            bool is_in_corridor = (D_right < MAX_CORRIDOR_WIDTH) && (D_left < MAX_CORRIDOR_WIDTH);

            if (is_in_corridor)
            {
                // Calculate total width and target center distance
                double corridor_width = D_right + D_left;
                double target_midpoint_distance = corridor_width / 2.0;

                // Error: D_left - target_midpoint_distance.
                // Positive error (D_left is larger): Robot is too far right -> needs negative (right) turn.
                double centering_error = D_left - target_midpoint_distance;
                
                // Apply Proportional control for steering
                angular_vel = -KP_STEER * centering_error;
                
                // Clamp angular velocity
                angular_vel = std::min(std::max(angular_vel, -MAX_ANGULAR_VEL), MAX_ANGULAR_VEL);
                
                RCLCPP_INFO(this->get_logger(), "W: %.2f, D_L: %.2f, Error: %.2f, Angular: %.2f", 
                            corridor_width, D_left, centering_error, angular_vel);
            }
            else
            {
                // Edge case: In a wide open space or junction, just move straight.
                angular_vel = 0.0;
                RCLCPP_INFO(this->get_logger(), "Path wide open. Moving straight.");
            }
        }

        // 4. Publish Command
        twist.angular.z = angular_vel;
        cmd_vel_pub_->publish(twist);
    }

    void stopRobot()
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MidPathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}