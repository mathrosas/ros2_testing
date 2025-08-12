#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// Action generated from your package's action/Waypoint.action
#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fastbot_as", options) {

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
        this,
        "fastbot_as", // action name changed
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    // Topics changed to FastBot
    this->cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel",
                                                          10);
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "fastbot_as action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  geometry_msgs::msg::Point position_;
  double yaw_{0.0};

  geometry_msgs::msg::Point des_pos_;
  double yaw_precision_ = M_PI / 90; // +/- 2 degree allowed
  double dist_precision_ = 0.05;

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
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position (%.3f, %.3f, %.3f)",
                goal->position.x, goal->position.y, goal->position.z);
    des_pos_ = goal->position;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(25);

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    bool success = true;

    while (rclcpp::ok()) {
      const double desired_yaw =
          std::atan2(des_pos_.y - position_.y, des_pos_.x - position_.x);
      const double err_pos = std::sqrt(std::pow(des_pos_.y - position_.y, 2) +
                                       std::pow(des_pos_.x - position_.x, 2));
      const double err_yaw = desired_yaw - yaw_;

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (err_pos > dist_precision_) {
        geometry_msgs::msg::Twist twist_msg;
        if (std::fabs(err_yaw) > yaw_precision_) {
          // fix yaw
          twist_msg.angular.z = (err_yaw > 0.0) ? 0.65 : -0.65;
          twist_msg.linear.x = 0.0;
        } else {
          // go straight
          twist_msg.linear.x = 0.6;
          twist_msg.angular.z = 0.0;
        }
        cmd_vel_publisher_->publish(twist_msg);

        feedback->position = position_;
        feedback->state = "moving";
        goal_handle->publish_feedback(feedback);
      } else {
        break;
      }

      loop_rate.sleep();
    }

    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop);

    if (success) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}