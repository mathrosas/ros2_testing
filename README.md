# FastBot Waypoints — Task 2 (ROS 2 Tests) — Updated for Current Code

This README matches your **current C++ action server** and **GTest node-level tests** for `fastbot_waypoints` (Checkpoint 23 - Testing, Task 2).  
It explains how to build, run the sim and server, and verify **pass**/**fail** outcomes that align with your code.

---

## 1) What this package includes

- **Action server node:** `fastbot_action_server` (C++).  
  Behavior: rotate in place to face the goal (±0.65 rad/s) until yaw error ≤ ~2°, then drive forward (`0.6 m/s`) until within **0.05 m** of the goal; stop and report success.
- **Custom action:** `action/Waypoint.action`.
- **Node-level tests (GTest):** `test/test_waypoints.cpp`  
  - **Final position** near goal (`EXPECT_NEAR` with current tolerance **±0.20 m**).  
  - **Final yaw** vs. **desired heading** computed from the *actual start pose* to the goal using `shortest_ang_diff` (tolerance currently **very lenient** to avoid flakes).

> **Action definition** (`action/Waypoint.action`)
```text
# Goal
geometry_msgs/Point position
---
# Result
bool success
---
# Feedback
geometry_msgs/Point position
string state
```

---

## 2) Package layout (reference)

```
fastbot_waypoints/
├─ CMakeLists.txt
├─ package.xml
├─ action/
│  └─ Waypoint.action
├─ src/
│  └─ fastbot_action_server.cpp
└─ test/
   └─ test_waypoints.cpp
```

**Exact names/topics used by your code & tests**
- **Action name:** `fastbot_as` (server must advertise exactly this)
- **Odometry topic:** `/fastbot/odom`
- **Velocity command topic:** `/fastbot/cmd_vel`

---

## 3) Build prerequisites

```bash
# From your ROS 2 workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Dependencies used by your code/tests: `rclcpp`, `rclcpp_action`, `geometry_msgs`, `nav_msgs`, **tf2** (for quaternion→yaw), and generated action types.

---

## 4) CMake & package.xml hints (updated)

> **Environment note (required in your setup):** whenever you modify `CMakeLists.txt`, include:
```cmake
# Link directories (macOS Homebrew example; harmless elsewhere)
link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)
```

**CMakeLists.txt** (key lines to adapt)
```cmake
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
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2)

# Generated types for the target
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS fastbot_action_server DESTINATION lib/${PROJECT_NAME})

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoints test/test_waypoints.cpp)
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs tf2)
  rosidl_target_interfaces(test_waypoints
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()
```

**package.xml** (key lines)
```xml
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
```

---

## 5) Launch the simulation (FastBot, ROS 2)

**Terminal 1**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```
If Gazebo misbehaves:
```bash
ps faux | grep gz
kill -9 <pid>
```
Then relaunch.

---

## 6) Run the action server

**Terminal 2**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

**What your node does**
- Subscribes to **`/fastbot/odom`** (QoS: best-effort, volatile).
- Publishes to **`/fastbot/cmd_vel`**.
- Serves action **`fastbot_as`**.  
- Controller thresholds: `yaw_precision_ = π/90` (~2°), `dist_precision_ = 0.05 m`.

---

## 7) Run the tests — default **PASS**

The client waits up to **10 s** for the action server, **60 s** for the result.  
Your current default goal in `test/test_waypoints.cpp` is:

```cpp
double goal_x = 2.00; // SUCCESS
double goal_y = 1.25; // SUCCESS
```

**Terminal 3**
```bash
cd ~/ros2_ws
colcon build && source install/setup.bash

# Show test output live:
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+

# Summarize:
colcon test-result --all
```

**Expected PASS:**
```
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

**What's asserted (as coded)**
- **Position:** `EXPECT_NEAR(..., error_margin)` with `error_margin = 0.20` (±20 cm).  
  > Note: The comment says “5 cm tolerance” but the code uses **0.20 m**.  
  > If you want to match the controller stop band (5 cm), set `error_margin = 0.05`.
- **Yaw:** compares actual yaw to the **desired heading** from *observed start (x,y)* → goal (x,y) using `shortest_ang_diff`.  
  Current tolerance is intentionally very lenient:
  ```cpp
  const double per_step = M_PI;
  const double tol = 10.0 * per_step; // 10π rad
  ```
  Tighten it for stricter grading (see §8B).

---

## 8) Force a **FAIL** (options that align with your code)

### A) Change the goal to a “bad” pair
At the goal definitions near the top of `test_waypoints.cpp`, replace:
```cpp
double goal_x = 2.00; // SUCCESS
double goal_y = 1.25; // SUCCESS
```
with one of your **known failing** pairs:
```cpp
double goal_x = 1.50; // FAIL
double goal_y = 2.00; // FAIL
// or
double goal_x = 0.50; // FAIL
double goal_y = 0.00; // FAIL
```

Then rebuild and re-run tests:
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

### B) Tighten the tolerances
- **Position:** in `TestEndPosition`, change:
  ```cpp
  double error_margin = 0.20; // → e.g., 0.05
  ```
- **Yaw:** in `TestEndYaw`, change:
  ```cpp
  const double per_step = M_PI/90.0; // 2 degrees
  const double tol = 10.0 * per_step; // ~20 degrees
  ```
  Or set `const double tol = 0.20;` (≈11.5°).  
  Smaller tolerances will convert marginal runs into test failures.

**Typical FAIL summary (example)**
```
Summary: 2 tests, 1 errors, 0 failures, 1 skipped
```

> Note: Your current tests **do not** include a `kForceFail` flag—use (A) or (B) to produce failing outcomes.

---

## 9) Troubleshooting

- **No odom received:** confirm `/fastbot/odom` exists and the sim is running.  
  `ros2 topic echo /fastbot/odom --once`
- **Action server unavailable:** ensure `fastbot_action_server` is running and advertising `fastbot_as`.  
  `ros2 action list`
- **Robot rotates forever:** verify quaternion→yaw conversion and that yaw signs match the sim; it will only move forward when `|err_yaw| ≤ ~2°`.
- **Tests hang:** launch order matters → **sim first**, then **server**, then **tests**.
- **Gazebo zombie:** `ps faux | grep gz` → `kill -9 <pid>` → relaunch.

---

## 10) Quick grading workflow

1) **Terminal 1 - Launch sim**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```

2) **Terminal 2 - Start action server**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

3) **Terminal 3 - PASS case (default goal 2.00, 1.25)**
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
# Expect: Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

4) **Terminal 3 - FAIL case**
   - Edit the goal to **(1.50, 2.00)** or **(0.50, 0.00)**, *or* tighten tolerances.  
   - Rebuild & re-run tests:
```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

---

**Done.** This README is aligned with your current controller (yaw-first, then go-to-point, 5 cm stop band) and your tests (dynamic heading check, default ±20 cm position tolerance, generous yaw tolerance). Adjust tolerances and/or goals as above to produce deterministic PASS/FAIL outcomes.
