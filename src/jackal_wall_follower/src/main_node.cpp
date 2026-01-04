#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stack>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

const double LINEAR_SPEED = 1.0;
const double ANGULAR_SPEED = 1.5;
const double STOP_DISTANCE = 0.9;
const double MAX_SENSOR_RANGE = 3.5;
const double WINDOW_DEGREES = 10.0;

const double KP = 0.5;
const double KI = 0.001;
const double KD = 0.05;

const double PATH_AVAILABLE_THRESHOLD = 2.0;
const double FRONT_BLOCKED_THRESHOLD = 1.2;

const int MAZE_DIM = 8;
const double CELL_SIZE = 1.8;

enum class RobotState {
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_BACK
};

struct coordinate {
  int x;
  int y;

  bool operator==(const coordinate& other) const {
    return x == other.x && y == other.y;
  }
};

struct intersection {
  int id;
  coordinate loc;

  bool left_open;
  bool right_open;
  bool front_open;
};

class ForwardWalkAlgorithm : public rclcpp::Node {
 public:
  ForwardWalkAlgorithm() : Node("forward_walk_node") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ForwardWalkAlgorithm::scanCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom/filtered", 10,
        std::bind(&ForwardWalkAlgorithm::odomCallback, this, _1));
    current_state_ = RobotState::MOVING_FORWARD;
    current_x_ = 0.0;
    current_y_ = 0.0;
    odom_received_ = false;
    target_turn_angle_ = M_PI / 2.0;
    std::string path = getLogPath();
    std::ofstream log_file(path, std::ios::trunc);
    if (log_file.is_open()) {
      log_file.close();
      RCLCPP_INFO(this->get_logger(), "Logging to: %s", path.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "PERMISSION ERROR: Cannot open %s",
                   path.c_str());
    }
    last_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Node started.");
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::stack<intersection> dfs_stack;
  std::vector<coordinate> history;

  RobotState current_state_;
  rclcpp::Time turn_start_time_;
  double target_turn_angle_ = M_PI / 2.0;
  double turn_duration;

  coordinate current_grid_pos_ = {-99, -99};
  coordinate last_logged_pos_ = {-99, -99};

  bool odom_received_;
  double current_x_;
  double current_y_;

  double prev_error_ = 0.0;
  double integral_ = 0.0;
  rclcpp::Time last_time_;

  std::string getLogPath() {
    const char* home = getenv("HOME");
    if (home == NULL) return "path_log.txt";
    return std::string(home) + "/ros2_ws/path_log.txt";
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    odom_received_ = true;

    int col_idx = std::max(0, std::min(int(current_x_ / 2.3), 15));
    int row_idx = std::max(0, std::min(int(current_y_ / 2.3), 15));

    current_grid_pos_ = {col_idx, row_idx};

    if (!(current_grid_pos_ == last_logged_pos_)) {
      double cell_boundary_x = col_idx * 2.3;
      double cell_boundary_y = row_idx * 2.3;

      double dist_in_x = current_x_ - cell_boundary_x;
      double dist_in_y = current_y_ - cell_boundary_y;

      bool stable_x = (dist_in_x > 0.05 && dist_in_x < (2.3 - 0.05));
      bool stable_y = (dist_in_y > 0.05 && dist_in_y < (2.3 - 0.05));

      if (stable_x && stable_y) {
        last_logged_pos_ = current_grid_pos_;

        saveCoordinateToFile(current_grid_pos_);

        RCLCPP_INFO(this->get_logger(), "Logged Grid: [%d, %d] (Row, Col)",
                    row_idx, col_idx);
      }
    }

    RCLCPP_INFO(this->get_logger(), "X: %.2f | Y: %.2f", current_x_, current_y_);
  }

  void saveCoordinateToFile(coordinate c) {
    std::string path = getLogPath();
    std::ofstream log_file(path, std::ios::app);

    if (log_file.is_open()) {
      log_file << c.x << "," << c.y << std::endl;
      log_file.close();
      RCLCPP_INFO(this->get_logger(), "!!! WROTE TO FILE: %d, %d !!!", c.x,
                  c.y);
    }
  }

  double getAverageRange(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                         double angle_rad) {
    double sum = 0.0;
    int count = 0;
    double half_window = (WINDOW_DEGREES / 2.0) * (M_PI / 180.0);
    double start_angle = angle_rad - half_window;
    double end_angle = angle_rad + half_window;

    int start_idx =
        static_cast<int>((start_angle - msg->angle_min) / msg->angle_increment);
    int end_idx =
        static_cast<int>((end_angle - msg->angle_min) / msg->angle_increment);

    start_idx = std::max(0, start_idx);
    end_idx = std::min(static_cast<int>(msg->ranges.size()) - 1, end_idx);

    for (int i = start_idx; i <= end_idx; ++i) {
      double r = msg->ranges[i];
      if (!std::isnan(r) && !std::isinf(r) && r >= msg->range_min &&
          r <= msg->range_max) {
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

  double calculatePID(double dt, double error) {
    integral_ += error * dt;
    integral_ = std::max(-1.0, std::min(1.0, integral_));

    double derivative = 0.0;
    if (dt > 0.0) {
      derivative = (error - prev_error_) / dt;
    }

    prev_error_ = error;

    return (KP * error) + (KI * integral_) + (KD * derivative);
  }

  void resetPID() {
    integral_ = 0;
    prev_error_ = 0;
  }

  // void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //   auto twist = geometry_msgs::msg::Twist();
  //   rclcpp::Time current_time = this->now();
    
  //   if (last_time_.nanoseconds() == 0) {
  //     last_time_ = current_time;
  //     return;
  //   }

  //   double dt = (current_time - last_time_).seconds();
  //   if (dt <= 0.0) {
  //     dt = 0.01;
  //   }

  //   double front_dist = getAverageRange(msg, 0.0);
  //   double front_left_dist = getAverageRange(msg, 45.0 * M_PI / 180.0);
  //   double front_right_dist = getAverageRange(msg, -45.0 * M_PI / 180.0);
    
  //   double left_dist = getAverageRange(msg, 90.0 * M_PI / 180.0);
  //   double right_dist = getAverageRange(msg, -90.0 * M_PI / 180.0);

  //   if (current_state_ == RobotState::MOVING_FORWARD) {
  //     bool left_open = front_left_dist > PATH_AVAILABLE_THRESHOLD;
  //     bool right_open = front_right_dist > PATH_AVAILABLE_THRESHOLD;
  //     bool front_blocked = front_dist < FRONT_BLOCKED_THRESHOLD;

  //     if (front_blocked) {
  //       if (right_open && !left_open) {
  //         target_turn_angle_ = M_PI / 2.0;
  //         turn_start_time_ = this->now();
  //         current_state_ = RobotState::TURNING_RIGHT;
  //         resetPID();
  //         RCLCPP_INFO(this->get_logger(), "Starting RIGHT turn");
  //       } else if (left_open && !right_open) {
  //         target_turn_angle_ = M_PI / 2.0;
  //         turn_start_time_ = this->now();
  //         current_state_ = RobotState::TURNING_LEFT;
  //         resetPID();
  //         RCLCPP_INFO(this->get_logger(), "Starting LEFT turn");
  //       } else if (left_open && right_open) {
  //         target_turn_angle_ = M_PI / 2.0;
  //         turn_start_time_ = this->now();
  //         current_state_ = RobotState::TURNING_LEFT;
  //         resetPID();
  //         RCLCPP_INFO(this->get_logger(), "Starting LEFT turn (T-junction)");
  //       } else if (front_dist <= STOP_DISTANCE) {
  //         target_turn_angle_ = M_PI;
  //         turn_start_time_ = this->now();
  //         current_state_ = RobotState::TURNING_BACK;
  //         resetPID();
  //         RCLCPP_INFO(this->get_logger(), "Starting U turn");
  //       }
  //     }

  //     if (current_state_ == RobotState::MOVING_FORWARD) {
  //       double error = left_dist - right_dist;
        
  //       twist.linear.x = LINEAR_SPEED;
  //       twist.angular.z = calculatePID(dt, error);
  //     }
  //   } else if (current_state_ == RobotState::TURNING_BACK) {
  //     double turn_duration = target_turn_angle_ / ANGULAR_SPEED;
  //     twist.linear.x = 0.0;
  //     twist.angular.z = ANGULAR_SPEED;
  //     if ((current_time - turn_start_time_).seconds() > turn_duration) {
  //       current_state_ = RobotState::MOVING_FORWARD;
  //     }
  //   } else if (current_state_ == RobotState::TURNING_LEFT) {
  //     double turn_duration = target_turn_angle_ / ANGULAR_SPEED;
  //     twist.linear.x = LINEAR_SPEED;
  //     twist.angular.z = ANGULAR_SPEED;
  //     if ((current_time - turn_start_time_).seconds() > turn_duration) {
  //       current_state_ = RobotState::MOVING_FORWARD;
  //     }
  //   } else if (current_state_ == RobotState::TURNING_RIGHT) {
  //     double turn_duration = target_turn_angle_ / ANGULAR_SPEED;
  //     twist.linear.x = LINEAR_SPEED;
  //     twist.angular.z = -ANGULAR_SPEED;
  //     if ((current_time - turn_start_time_).seconds() > turn_duration) {
  //       current_state_ = RobotState::MOVING_FORWARD;
  //     }
  //   }

  //   last_time_ = current_time;
  //   cmd_vel_pub_->publish(twist);
  // }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto twist = geometry_msgs::msg::Twist();
    rclcpp::Time current_time = this->now();
    
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      dt = 0.01;
    }

    double front_dist = getAverageRange(msg, 0.0);
    double front_left_dist = getAverageRange(msg, 45.0 * M_PI / 180.0);
    double front_right_dist = getAverageRange(msg, -45.0 * M_PI / 180.0);
    
    double left_dist = getAverageRange(msg, 90.0 * M_PI / 180.0);
    double right_dist = getAverageRange(msg, -90.0 * M_PI / 180.0);

    if (current_state_ == RobotState::MOVING_FORWARD) {
      double error = left_dist - right_dist;
      twist.linear.x = LINEAR_SPEED;
      if (-0.75 < error && error < 0.75) {
        twist.angular.z = calculatePID(dt, error);
      }
      else {
        twist.angular.z = 0.0;
      }

      bool left_open = left_dist > PATH_AVAILABLE_THRESHOLD;
      bool right_open = right_dist > PATH_AVAILABLE_THRESHOLD;
      bool front_blocked = front_dist < FRONT_BLOCKED_THRESHOLD;

      if (front_blocked) {
        twist.linear.x = 0.0;
        if (right_open && !left_open) {
          current_state_ = RobotState::TURNING_RIGHT;
          turn_start_time_ = this->now();
          target_turn_angle_ = M_PI / 2;
          turn_duration = target_turn_angle_ / ANGULAR_SPEED;
          resetPID();
        }
        else if (left_open && !right_open) {
          current_state_ = RobotState::TURNING_LEFT;
          turn_start_time_ = this->now();
          target_turn_angle_ = M_PI / 2;
          turn_duration = target_turn_angle_ / ANGULAR_SPEED;
          resetPID();
        }
        else if (left_open && right_open) { // need some update
          current_state_ = RobotState::TURNING_RIGHT;
          turn_start_time_ = this->now();
          target_turn_angle_ = M_PI / 2;
          turn_duration = target_turn_angle_ / ANGULAR_SPEED;
          resetPID();
        }
        else if (!left_open && !right_open) {
          current_state_ = RobotState::TURNING_BACK;
          turn_start_time_ = this->now();
          target_turn_angle_ = M_PI / 2;
          turn_duration = target_turn_angle_ / ANGULAR_SPEED;
          resetPID();
        }
      }
    }
    else if (current_state_ == RobotState::TURNING_RIGHT) {
      twist.linear.x = 0.0;
      twist.angular.z = -ANGULAR_SPEED;
      if ((current_time - turn_start_time_).seconds() > turn_duration) {
        current_state_ = RobotState::MOVING_FORWARD;
      }
    }
    else if (current_state_ == RobotState::TURNING_LEFT) {
      twist.linear.x = 0.0;
      twist.angular.z = ANGULAR_SPEED;
      if ((current_time - turn_start_time_).seconds() > turn_duration) {
        current_state_ = RobotState::MOVING_FORWARD;
      }
    }
    else {
      twist.linear.x = 0.0;
      twist.angular.z = ANGULAR_SPEED;
      if ((current_time - turn_start_time_).seconds() > turn_duration) {
        current_state_ = RobotState::MOVING_FORWARD;
      }
    }

    last_time_ = current_time;
    cmd_vel_pub_->publish(twist);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardWalkAlgorithm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}