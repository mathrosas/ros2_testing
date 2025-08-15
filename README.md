# FastBot Waypoints — Task 2 (ROS 2 Tests, updated)

This README reflects your **current controller and tests**. It explains how to build, run, and test the 'fastbot_waypoints' package for **Checkpoint 23 - Testing (Task 2)**, including **clear pass/fail recipes that match your updated code**.

---

## 1) What this package contains

- **Action server node:** 'fastbot_action_server' (C++).  
  Behavior: fixes yaw toward the goal, then drives forward until within distance tolerance, then stops.
- **Custom action:** 'action/Waypoint.action'.
- **GTest node-level tests:** 'test/test_waypoints.cpp'  
  - **End position** '[x,y]' within tolerance.  
  - **Final yaw** compared to the desired heading computed from the *actual start pose* to the goal.

> **Action definition** ('action/Waypoint.action')
'''text
# Goal
geometry_msgs/Point position
---
# Result
bool success
---
# Feedback
geometry_msgs/Point position
string state
'''

---

## 2) Package layout (reference)

'''
fastbot_waypoints/
├─ CMakeLists.txt
├─ package.xml
├─ action/
│  └─ Waypoint.action
├─ src/
│  └─ fastbot_action_server.cpp
└─ test/
   └─ test_waypoints.cpp
'''

**Exact names/topics the tests expect**
- **Action name:** 'fastbot_as' (server must advertise this exact name)
- **Odom topic:** '/fastbot/odom'
- **(Typical) cmd_vel:** '/fastbot/cmd_vel' (used by your server)

---

## 3) Your current node logic (summary)

'fastbot_action_server.cpp':
- Subscribes to **'/fastbot/odom'** with best-effort QoS.
- Publishes Twist to **'/fastbot/cmd_vel'**.
- Accepts goals on **'fastbot_as'** (type 'fastbot_waypoints/Waypoint').
- Controller loop:
  - Compute 'desired_yaw = atan2(y_err, x_err)' and 'err_yaw = shortest_ang_diff(desired_yaw, yaw_)'.
  - If '|err_yaw| > yaw_precision_' (≈ ±2°), **rotate in place** (±0.65 rad/s).
  - Else **drive forward** ('0.6 m/s') straight toward the goal.
  - Stop when distance error ≤ 'dist_precision_ = 0.05 m'.
- Prints helpful logs: _Current Yaw_, _Desired Yaw_, _Error Yaw_, and state (“fix yaw” / “go to point”).

---

## 4) Build prerequisites

'''bash
# Work from your ROS 2 workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
'''

The node/tests use at least: 'rclcpp', 'rclcpp_action', 'geometry_msgs', 'nav_msgs', **tf2** (for quaternion → yaw).

---

## 5) Minimal CMake & package.xml hints

> **Important (Environment note):** Whenever you modify 'CMakeLists.txt', include this line (as requested in your environment):
'''cmake
# Link directories (macOS Homebrew example; harmless elsewhere)
link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)
'''

**CMakeLists.txt** (key lines to adapt)
'''cmake
cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Generate the action interface
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server rclcpp rclcpp_action geometry_msgs nav_msgs tf2)
# Ensure generated types are visible
rosidl_target_interfaces(fastbot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS fastbot_action_server DESTINATION lib/${PROJECT_NAME})

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoints test/test_waypoints.cpp)
  ament_target_dependencies(test_waypoints rclcpp rclcpp_action geometry_msgs nav_msgs tf2)
  rosidl_target_interfaces(test_waypoints ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()
'''

**package.xml** (key lines)
'''xml
<package format="3">
  <name>fastbot_waypoints</name>
  <version>0.0.1</version>
  <description>Waypoint action server and tests for FastBot</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_cmake_gtest</test_depend>
</package>
'''

---

## 6) Launch the simulation (FastBot, ROS 2)

Terminal 1:
'''bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
'''

If Gazebo misbehaves:
'''bash
ps faux | grep gz
kill -9 <pid>
'''
Then relaunch.

---

## 7) Run the action server

Terminal 2:
'''bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
'''

---

## 8) Run the tests — **Passing conditions**

The test client waits up to **10s** for the server and **60s** for the result.  
Default **goal** values in your test file ('test/test_waypoints.cpp') are:

'''cpp
double goal_x = 2.00; // SUCCESS
double goal_y = 1.25; // SUCCESS
'''

Run:
'''bash
cd ~/ros2_ws
colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
'''

**What’s checked (as coded now):**
- **End position** uses 'EXPECT_NEAR' with 'error_margin = 0.20' (i.e., ±0.20 m).  
- **End yaw** compares the actual yaw to the **desired heading** computed from your **actual start pose** to the goal, using the shortest signed angle difference.  
  > Current tolerance is **very lenient** in code ('tol = 10π' rad) to avoid false negatives.

**Expected PASS summary:**
'''
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
'''

---

## 9) Run the tests — **Failing conditions (matching your code)**

You have two straightforward ways to force a failure:

### A) Change the goal to one of your known “bad” pairs
At the top of 'test/test_waypoints.cpp', replace:
'''cpp
double goal_x = 2.00; // SUCCESS
double goal_y = 1.25; // SUCCESS
'''
with one of these **fail** examples you annotated:
'''cpp
double goal_x = 1.50; // FAIL
double goal_y = 2.00; // FAIL
// or
double goal_x = 0.50; // FAIL
double goal_y = 0.00; // FAIL
'''

Then rebuild and re-run tests:
'''bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
'''

### B) Tighten tolerances in the assertions
- **Position:** change 'double error_margin = 0.20;' to something smaller, e.g. '0.02' (2 cm).
- **Yaw:** reduce tolerance by setting:
  '''cpp
  const double per_step = M_PI/90.0; // 2 degrees
  const double tol = 10.0 * per_step; // ~20 degrees total tolerance
  '''
  (Or set 'tol' directly, e.g. '0.20' rad.)

Either (A) or (B) will yield a **FAIL** consistent with your current controller and map.

**Typical FAIL summary (example):**
'''
Summary: 2 tests, 1 errors, 0 failures, 1 skipped
'''

---

## 10) Notes on the updated yaw test

- The yaw expectation is **computed dynamically** from the observed start pose → goal, so it’s robust to different spawn positions/orientations.
- Comparison uses 'shortest_ang_diff' to avoid wrap-around issues.
- By default, the tolerance is intentionally large to reduce map-specific flakes; tighten it when you need a strict check.

---

## 11) Troubleshooting

- **No odom received:** Confirm topic is '/fastbot/odom' and the sim is running.
- **Action server unavailable:** Ensure you started 'fastbot_action_server' and that it advertises **'fastbot_as'**.
- **Tests hang:** Start **simulation first**, then the **action server**, then run tests.
- **Robot spins but won't move forward:** Yaw error must be ≤ ~2° before it goes straight; verify the odometry orientation and sign conventions.

---

## 12) Quick grading workflow

1) **Terminal 1 - Launch sim**
'''bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
'''

2) **Terminal 2 - Start action server**
'''bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
'''

3) **Terminal 3 - PASS case (default goal 2.00, 1.25)**
'''bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
# Expect: Summary: 2 tests, 0 errors, 0 failures, 0 skipped
'''

4) **Terminal 3 - FAIL case**
   - Edit 'test/test_waypoints.cpp' goal to **(1.50, 2.00)** or **(0.50, 0.00)**, _or_ tighten tolerances as described.  
   - Rebuild & test:
'''bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
'''

---

**Done.** This README matches your current server and tests, preserving the exact names and topics your code uses, and provides reliable pass/fail paths aligned with your annotations.
