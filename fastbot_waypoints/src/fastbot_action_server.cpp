#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

static inline double shortest_ang_diff(double a, double b) {
  // shortest signed angle from b to a, in [-pi, pi]
  const double d = a - b;
  return std::atan2(std::sin(d), std::cos(d));
}

class WaypointActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fastbot_as", options) {

    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);

    auto odom_qos = rclcpp::QoS(50).best_effort().durability_volatile();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", odom_qos,
        std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(get_logger(), "fastbot_as action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point position_;
  double yaw_{0.0};

  geometry_msgs::msg::Point des_pos_;
  const double yaw_precision_ = M_PI / 90.0; // +/- 2 degrees
  const double dist_precision_ = 0.05;       // 5 cm

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID & /*uuid*/,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(get_logger(), "goal (%.3f, %.3f, %.3f) received",
                goal->position.x, goal->position.y, goal->position.z);
    des_pos_ = goal->position;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> ) {
    RCLCPP_INFO(get_logger(), "The goal has been cancelled/preempted");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    rclcpp::Rate rate(25);
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();
    bool success = true;

    while (rclcpp::ok()) {
      const double desired_yaw =
          std::atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
      const double err_pos =
          std::hypot(des_pos_.y - position_.y, des_pos_.x - position_.x);
      const double err_yaw = shortest_ang_diff(desired_yaw, yaw_);

      RCLCPP_INFO(get_logger(), "Current Yaw: %.5f", yaw_);
      RCLCPP_INFO(get_logger(), "Desired Yaw: %.5f", desired_yaw);
      RCLCPP_INFO(get_logger(), "Error Yaw: %.5f", err_yaw);

      if (goal_handle->is_canceling()) {
        publish_stop();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      if (err_pos > dist_precision_) {
        geometry_msgs::msg::Twist cmd;
        if (std::fabs(err_yaw) > yaw_precision_) {
          // fix yaw
          RCLCPP_INFO(get_logger(), "fix yaw");
          cmd.angular.z = (err_yaw > 0.0) ? 0.65 : -0.65;
          cmd.linear.x = 0.0;
          feedback->state = "fix yaw";
        } else {
          // go to point
          RCLCPP_INFO(get_logger(), "go to point");
          cmd.linear.x = 0.6;
          cmd.angular.z = 0.0;
          feedback->state = "go to point";
        }
        cmd_vel_pub_->publish(cmd);

        feedback->position = position_;
        goal_handle->publish_feedback(feedback);
      } else {
        break; // reached target
      }

      rate.sleep();
    }

    publish_stop();

    if (success) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
  }

  void publish_stop() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
