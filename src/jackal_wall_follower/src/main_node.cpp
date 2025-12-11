#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

const double LINEAR_SPEED = 0.8;
const double ANGULAR_SPEED = 1.4;
const double STOP_DISTANCE = 0.9;
const double MAX_SENSOR_RANGE = 3.5;
const double WINDOW_DEGREES = 10.0;
const double CENTERING_KP = 0.5;

enum class RobotState {
  MOVING_FORWARD,
  TURNING
};

class forwardWalkAlgorithm : public rclcpp::Node {
public:
  forwardWalkAlgorithm() : Node("forward") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&forwardWalkAlgorithm::scanCallback, this, _1));
    current_state_ = RobotState::MOVING_FORWARD;
    turn_direction_ = 1;
    target_turn_angle_ = 0.0;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  RobotState current_state_;
  rclcpp::Time turn_start_time_;
  int turn_direction_;
  double target_turn_angle_;

  double getAverageRange(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_rad) {
    double sum = 0.0;
    int count = 0;
    
    double half_window = (WINDOW_DEGREES / 2.0) * (M_PI / 180.0);
    
    double start_angle = angle_rad - half_window;
    double end_angle = angle_rad + half_window;

    int start_idx = static_cast<int>((start_angle - msg->angle_min) / msg->angle_increment);
    int end_idx = static_cast<int>((end_angle - msg->angle_min) / msg->angle_increment);

    start_idx = std::max(0, start_idx);
    end_idx = std::min(static_cast<int>(msg->ranges.size()) - 1, end_idx);

    for (int i = start_idx; i <= end_idx; ++i) {
      double r = msg->ranges[i];
      
      if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min && r <= msg->range_max) {
        sum += r;
        count++;
      } else if (std::isinf(r)) {
        sum += MAX_SENSOR_RANGE;
        count++;
      }
    }

    if (count == 0) return 0.0;
    return sum / count;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
    auto twist = geometry_msgs::msg::Twist();

    double front_dist = getAverageRange(msg, 0.0);
    double left_dist = getAverageRange(msg, 45.0 * M_PI / 180.0);
    double right_dist = getAverageRange(msg, -45.0 * M_PI / 180.0);

    if (current_state_ == RobotState::MOVING_FORWARD) {
      if (front_dist > STOP_DISTANCE) {
        twist.linear.x = LINEAR_SPEED;
        double error = left_dist - right_dist;
        twist.angular.z = CENTERING_KP * error;
      } else {
        current_state_ = RobotState::TURNING;
        turn_start_time_ = this->now();
        
        bool is_left_open = left_dist > STOP_DISTANCE;
        bool is_right_open = right_dist > STOP_DISTANCE;

        if (!is_left_open && !is_right_open) {
          target_turn_angle_ = M_PI;
          turn_direction_ = 1;
        } else {
          target_turn_angle_ = M_PI / 2.0;
          if (left_dist > right_dist) {
            turn_direction_ = 1;
          } else {
            turn_direction_ = -1;
          }
        }
      }
    } else if (current_state_ == RobotState::TURNING) {
      double turn_duration = target_turn_angle_ / ANGULAR_SPEED;
      if ((this->now() - turn_start_time_).seconds() < turn_duration) {
        twist.linear.x = 0.0;
        twist.angular.z = turn_direction_ * ANGULAR_SPEED;
      } else {
        current_state_ = RobotState::MOVING_FORWARD;
      }
    }

    cmd_vel_pub_->publish(twist);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<forwardWalkAlgorithm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}