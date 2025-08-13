# FastBot Waypoints — Task 2 (ROS 2 Tests)

This README explains how to build, run, and **test** the ROS 2 package `fastbot_waypoints` for **Checkpoint 23 - Testing (Task 2)**. It includes **clear instructions for passing and failing conditions** for the GTest-based node-level tests.

---

## 1) What you build in Task 2

- A ROS 2 C++ action server node: **`fastbot_action_server`**, which makes FastBot drive to a 2D waypoint.
- A custom action: **`action/Waypoint.action`**.
- GTest node-level tests: **`test/test_waypoints.cpp`** that check:
  1. Final **position** `[x, y]` is within tolerance.
  2. Final **yaw** (heading) is within tolerance.

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

**Topics & names used by tests**  
- Odom: **`/fastbot/odom`**  
- Action name: **`fastbot_as`** (the server must advertise this exact name)

---

## 3) Build prerequisites

```bash
# Always work from your ROS 2 workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Your node will depend on (at least): `rclcpp`, `rclcpp_action`, `geometry_msgs`, `nav_msgs`, and the generated action types from this package.

---

## 4) Minimal CMake & package.xml hints

> **Important (Environment note):** Whenever you modify `CMakeLists.txt`, include this line (as requested in your environment):
```cmake
# Link directories (macOS Homebrew example; harmless elsewhere)
link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)
```

**CMakeLists.txt** (key lines only; adapt to your project)
```cmake
cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate the action interface
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server rclcpp rclcpp_action geometry_msgs nav_msgs)

# Make sure the node can see the generated headers
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS fastbot_action_server DESTINATION lib/${PROJECT_NAME})

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoints test/test_waypoints.cpp)
  target_link_libraries(test_waypoints)
  ament_target_dependencies(test_waypoints rclcpp rclcpp_action geometry_msgs nav_msgs)
  rosidl_target_interfaces(test_waypoints ${PROJECT_NAME} "rosidl_typesupport_cpp")
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

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_cmake_gtest</test_depend>
</package>
```

---

## 5) Launch the simulation (FastBot, ROS 2)

Terminal 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```
If Gazebo misbehaves:
```bash
ps faux | grep gz
kill -9 <process_id>
```
Then relaunch.

---

## 6) Run the action server (your node)

Terminal 2:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

**Your node must:**
- Subscribe to **`/fastbot/odom`**.
- Command the robot appropriately (e.g., `/fastbot/cmd_vel` if you use it).
- Advertise the action **`fastbot_as`** of type `fastbot_waypoints/action/Waypoint`.

---

## 7) Run the tests — **Passing conditions**

> **Use case (success):** The **current test code** in `test/test_waypoints.cpp` is configured for **success** out of the box.

Terminal 3:
```bash
cd ~/ros2_ws
colcon build && source install/setup.bash

# Show test output live:
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+

# Summarize:
colcon test-result --all
```

**Expected summary for PASS:**
```
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

---

## 8) Run the tests — **Failing conditions**

> **Use case (fail, as requested):** To **force a failure**, edit the *EndPositionWithinTolerance* test and change the goal to **`(0.0, 0.0)`**. That goal will **fail** under the default setup.

Steps:

1) Open `test/test_waypoints.cpp` and find:
```cpp
goal.position.x = 0.50;
goal.position.y = -0.90;
```
2) Replace with:
```cpp
goal.position.x = 0.0;
goal.position.y = 0.0;
```
3) Rebuild & rerun tests:
```bash
cd ~/ros2_ws
colcon build && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected summary for FAIL (example from the grading guide):**
```
Summary: 2 tests, 1 errors, 0 failures, 1 skipped
```

> **Alternative (optional):** The file defines a `kForceFail` toggle. Setting it to `true` will deliberately provoke failures by tightening/altering expectations. However, for grading, the **recommended** failing case here is simply testing position **`(0.0, 0.0)`** as described above.

---

## 9) What the tests do (quick notes)

- **EndPositionWithinTolerance**  
  Sends a reachable goal (default: `(0.50, -0.90)`) to `fastbot_as`, waits for `SUCCEEDED`, then checks the final odometry `[x, y]` against the expected values with a tolerance (default `±0.10`).  
  **Fail recipe you asked for:** set goal to `(0.0, 0.0)`.

- **EndYawWithinTolerance**  
  Samples odometry orientation and compares yaw to a fixed expected value (`1.462` rad) with tolerance (default `±0.20`).  
  If your spawn orientation differs, adjust the constant:
  ```cpp
  constexpr double kExpectedYaw = 1.462; // change if your map/spawn differs
  ```

---

## 10) Troubleshooting checklist

- **No odom received:** Ensure the subscription uses **`/fastbot/odom`** (matches the sim).  
- **Action server not available:** Confirm your node advertises **`fastbot_as`** and you started it in Terminal 2.  
- **Tests hang:** The sim must be running **before** you execute tests, and the action server must be up.  
- **Gazebo zombie:** `ps faux | grep gz` → `kill -9 <pid>` → relaunch.

---

## 11) Quick grading workflow (summary)

1. **Terminal 1 - Launch sim**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch fastbot_gazebo one_fastbot_room.launch.py
   ```

2. **Terminal 2 - Run action server**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run fastbot_waypoints fastbot_action_server
   ```

3. **Terminal 3 - PASS case (default code)**
   ```bash
   cd ~/ros2_ws && colcon build && source install/setup.bash
   colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
   colcon test-result --all
   # Expect: Summary: 2 tests, 0 errors, 0 failures, 0 skipped
   ```

4. **Terminal 3 - FAIL case**
   - Edit `test/test_waypoints.cpp` → goal `(0.0, 0.0)`  
   - Rebuild & test:
   ```bash
   cd ~/ros2_ws && colcon build && source install/setup.bash
   colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
   colcon test-result --all
   # Expect: Summary: 2 tests, 1 errors, 0 failures, 1 skipped
   ```

---

**That's it.** This README contains everything needed to **run the simulator, start the action server, and verify both passing and failing test outcomes** for Task 2.
