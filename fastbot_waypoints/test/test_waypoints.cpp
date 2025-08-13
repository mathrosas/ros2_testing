#include <chrono>
#include <gtest/gtest.h>
#include <memory>

#include "fastbot_waypoints/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using WaypointAction = fastbot_waypoints::action::Waypoint;

static const bool kForceFail =
    false; // <-- set to true to get the "failing conditions" for grading

class WaypointsFixture : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("fastbot_waypoints_test");
    client_ = rclcpp_action::create_client<WaypointAction>(node_, "fastbot_as");

    // odom sub (best effort to match sim)
    auto qos = rclcpp::QoS(50).best_effort().durability_volatile();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          last_odom_ = *msg;
          got_odom_ = true;
        });

    // Wait for server
    ASSERT_TRUE(client_->wait_for_action_server(10s))
        << "Action server not available";
    // Warm up odom
    const auto start = node_->now();
    while (!got_odom_ && (node_->now() - start) < rclcpp::Duration(10, 0)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(20ms);
    }
    ASSERT_TRUE(got_odom_) << "Didn't receive /odom before starting tests";
  }

  void TearDown() override {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<WaypointAction>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry last_odom_{};
  bool got_odom_{false};
};

TEST_F(WaypointsFixture, EndPositionWithinTolerance) {
  // Send a small reachable goal from typical spawn (0,0,0)
  WaypointAction::Goal goal;
  goal.position.x = 0.50;
  goal.position.y = -0.75;

  auto send_goal_options =
      rclcpp_action::Client<WaypointAction>::SendGoalOptions{};
  auto future_handle = client_->async_send_goal(goal, send_goal_options);
  rclcpp::spin_until_future_complete(node_, future_handle, 60s);
  auto goal_handle = future_handle.get();
  ASSERT_TRUE(goal_handle) << "Failed to send goal";

  auto future_result = client_->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node_, future_result, 120s);
  auto result = future_result.get();
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
      << "Action did not succeed";
  ASSERT_TRUE(result.result->success);

  // Get final odom (give it a moment to settle)
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(50ms);
  }

  const double x = last_odom_.pose.pose.position.x;
  const double y = last_odom_.pose.pose.position.y;

  const double expected_x = kForceFail ? 0.50 : 0.50;
  const double expected_y =
      kForceFail
          ? 0.30
          : -0.75; // force fail by expecting wrong Y when kForceFail=true
  const double tol = kForceFail ? 0.01 : 0.10;

  EXPECT_NEAR(x, expected_x, tol);
  EXPECT_NEAR(y, expected_y, tol);
}

TEST_F(WaypointsFixture, EndYawWithinTolerance) {
  // Don’t send a goal; just check current yaw against 1.462
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(50ms);
  }

  constexpr double kExpectedYaw = 1.462; // target for THIS test
  const double tol = kForceFail ? 0.02 : 0.20;

  const auto &q = last_odom_.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  while (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  while (yaw < -M_PI)
    yaw += 2.0 * M_PI;

  EXPECT_NEAR(yaw, kExpectedYaw, tol);
}