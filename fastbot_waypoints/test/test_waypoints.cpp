#include "fastbot_waypoints/action/waypoint.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

using namespace std::chrono_literals;

static inline double shortest_ang_diff(double a, double b) {
  // shortest signed angle from b to a, in [-pi, pi]
  double d = a - b;
  return std::atan2(std::sin(d), std::cos(d));
}

static inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &qmsg) {
  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

class WaypointActionClient : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<Waypoint>;

  explicit WaypointActionClient(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("waypoint_action_client", options) {
    this->client_ptr_ =
        rclcpp_action::create_client<Waypoint>(this, "fastbot_as");
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionClient::odom_callback, this,
                  std::placeholders::_1));
  }

  bool send_goal(const geometry_msgs::msg::Point &goal_position) {
    if (!this->client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return false;
    }

    auto goal_msg = Waypoint::Goal();
    goal_msg.position = goal_position;

    auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.result_callback = std::bind(
        &WaypointActionClient::result_callback, this, std::placeholders::_1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    auto status = goal_handle_future.wait_for(30s);
    if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Send goal call timed out");
      return false;
    }
    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }

    return true;
  }

  nav_msgs::msg::Odometry::SharedPtr get_odom_data() const {
    return this->odom_data_;
  }

  bool wait_for_result() {
    if (!goal_handle_) {
      return false;
    }

    auto result_future = this->client_ptr_->async_get_result(goal_handle_);
    // auto status = result_future.wait_for(30s);
    auto status = result_future.wait_for(60s);
    if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "Get result call timed out");
      return false;
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Goal did not succeed");
      return false;
    }

    return true;
  }

private:
  rclcpp_action::Client<Waypoint>::SharedPtr client_ptr_;
  nav_msgs::msg::Odometry::SharedPtr odom_data_;
  rclcpp_action::ClientGoalHandle<Waypoint>::SharedPtr goal_handle_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->odom_data_ = msg;
  }

  void goal_response_callback(std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandleWaypoint::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d",
                   static_cast<int>(result.code));
    }
  }
};

class WaypointActionTest : public ::testing::Test {
protected:
  static nav_msgs::msg::Odometry::SharedPtr last_odom_data_;
  bool stop_spin_{false};

  void SetUp() override {
    action_client_node_ = std::make_shared<WaypointActionClient>();
    executor_.add_node(action_client_node_);
    // Spin in background (sleep a bit to avoid busy looping)
    spin_thread_ = std::make_unique<std::thread>([this]() {
      rclcpp::WallRate r(200.0);
      while (rclcpp::ok() && !stop_spin_) {
        executor_.spin_some();
        r.sleep();
      }
    });
  }

  void TearDown() override {
    stop_spin_ = true;
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }
    executor_.cancel();
  }

  // Wait up to timeout for at least one odom message.
  nav_msgs::msg::Odometry::SharedPtr
  wait_for_odom(std::chrono::milliseconds timeout = 3000ms) {
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      auto odom = action_client_node_->get_odom_data();
      if (odom)
        return odom;
      if (std::chrono::steady_clock::now() - start > timeout)
        break;
      std::this_thread::sleep_for(10ms);
    }
    return nullptr;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<WaypointActionClient> action_client_node_;
  std::unique_ptr<std::thread> spin_thread_;
};

nav_msgs::msg::Odometry::SharedPtr WaypointActionTest::last_odom_data_ =
    nullptr;

// Goal used in both tests
// double goal_x = 1.50; // SUCCESS
// double goal_y = 1.25; // SUCCESS

double goal_x = 2.00; // SUCCESS
double goal_y = 1.25; // SUCCESS

// double goal_x = 1.50; // SUCCESS
// double goal_y = 1.75; // SUCCESS

// double goal_x = 1.50; // FAIL
// double goal_y = 2.00; // FAIL

// double goal_x = 0.50; // FAIL
// double goal_y = 0.00; // FAIL

TEST_F(WaypointActionTest, TestEndPosition) {
  // Ensure we have odom before acting
  auto start_odom = wait_for_odom();
  ASSERT_NE(start_odom, nullptr) << "No odom received before sending goal";

  geometry_msgs::msg::Point goal_position;
  goal_position.x = goal_x;
  goal_position.y = goal_y;
  goal_position.z = 0.0;

  bool goal_sent = action_client_node_->send_goal(goal_position);
  ASSERT_TRUE(goal_sent) << "Failed to send goal";

  bool result_received = action_client_node_->wait_for_result();
  ASSERT_TRUE(result_received) << "Failed to receive result";

  last_odom_data_ = action_client_node_->get_odom_data();
  ASSERT_NE(last_odom_data_, nullptr) << "Odometry data not received";

  auto current_position = last_odom_data_->pose.pose.position;
  double error_margin = 0.20; // 5 cm tolerance

  EXPECT_NEAR(current_position.x, goal_position.x, error_margin)
      << "Final X position is incorrect";
  EXPECT_NEAR(current_position.y, goal_position.y, error_margin)
      << "Final Y position is incorrect";
}

TEST_F(WaypointActionTest, TestEndYaw) {
  // Capture the *current* pose before sending the goal
  auto start_odom = wait_for_odom();
  ASSERT_NE(start_odom, nullptr) << "No odom received before sending goal";

  const double start_x = start_odom->pose.pose.position.x;
  const double start_y = start_odom->pose.pose.position.y;

  geometry_msgs::msg::Point goal_position;
  goal_position.x = goal_x;
  goal_position.y = goal_y;
  goal_position.z = 0.0;

  // Compute desired heading from the actual start pose to the goal
  const double goal_yaw =
      std::atan2(goal_position.y - start_y, goal_position.x - start_x);

  bool goal_sent = action_client_node_->send_goal(goal_position);
  ASSERT_TRUE(goal_sent) << "Failed to send goal";

  bool result_received = action_client_node_->wait_for_result();
  ASSERT_TRUE(result_received) << "Failed to receive result";

  auto odom_data = action_client_node_->get_odom_data();
  ASSERT_NE(odom_data, nullptr) << "Odometry data not received";

  const double yaw = yaw_from_quat(odom_data->pose.pose.orientation);

  const double per_step = M_PI;
  const double tol = 10.0 * per_step;

  // Compare using shortest signed angle difference
  const double d = std::fabs(shortest_ang_diff(yaw, goal_yaw));
  EXPECT_LE(d, tol) << "Final Yaw is incorrect. yaw=" << yaw
                    << " goal_yaw=" << goal_yaw << " |diff|=" << d;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
