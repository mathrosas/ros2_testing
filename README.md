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
├─ action/
│  └─ Waypoint.action
├─ include/fastbot_waypoints/
├─ src/
│  └─ fastbot_action_server.cpp          # Your C++ action server
├─ test/
│  └─ test_waypoints.cpp                 # GTest node-level tests
├─ CMakeLists.txt
└─ package.xml
```

**Topics used by the provided tests:**
- Action server name: **`fastbot_as`**
- Odometry topic: **`/fastbot/odom`**

Make sure your node publishes/subscribes consistently with these names (or adapt the test accordingly).

---

## 3) Build prerequisites

You should already have the FastBot simulation and dependencies installed. Typical dependencies for this package include:
- `rclcpp`, `rclcpp_action`
- `geometry_msgs`, `nav_msgs`
- `tf2`, `tf2_geometry_msgs`, `tf2_eigen` (or at least `tf2` core)
- `gtest` (brought in via `ament_cmake_gtest`)

> Build tools:
> - **colcon** (ROS 2 standard build tool)
> - **CMake** via `ament_cmake`

---

## 4) Launch the simulation (ROS 2)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```
If Gazebo misbehaves, stop it and relaunch. If a `gzserver` process remains, kill it with:
```bash
ps faux | grep gz
kill -9 <process_id>
```

---

## 5) Run the action server

Open a **new terminal**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

Leave this terminal running (the tests assume the server is available).

---

## 6) Build and run the tests (GTest)

Open a **third terminal**:

```bash
cd ~/ros2_ws
colcon build --packages-select fastbot_waypoints
source install/setup.bash

# Run only this package's tests and print live output
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+

# Summarize results
colcon test-result --all
```

**Expected output for passing conditions:**
```
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

**Expected output for failing conditions (at least one test made to fail on purpose):**
```
Summary: 2 tests, 1 errors, 0 failures, 1 skipped
```
(The exact numbers may vary slightly depending on how you force failures—details below.)

---

## 7) How the tests work

File: `test/test_waypoints.cpp`

- The test node **sends one goal** to the action server and waits for success.
- It continuously **listens to `/fastbot/odom`** to read the robot's final pose.
- **Test 1 - End Position**: compares final `[x, y]` with the goal, within a fixed tolerance.
- **Test 2 - End Yaw**: computes the desired yaw from the start pose toward the goal and compares it to the final yaw, within a tolerance.

### Tolerances (as written in your file)
- Position tolerance: `error_margin = 0.20` (meters). *(Note: the comment says “5 cm”, but `0.20` is **20 cm**.)*
- Yaw tolerance (in the provided snippet): `tol = 10 * M_PI`, which is **very large** (effectively always passing, since the shortest angle difference is ≤ π). See §8.3 for creating a yaw failure.

---

## 8) Pass/Fail scenarios you can reproduce

Below are **ready-to-use goal coordinate sets** from your `test_waypoints.cpp` comments and **three independent ways** to force failures (position mismatch, yaw mismatch, timeout).

### 8.1 Edit the goal coordinates
Open `test/test_waypoints.cpp` and find the block:
```cpp
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
```
Uncomment/choose one pair at a time, rebuild, and rerun the tests:

- **Passing cases** (use any one):
  - `(x, y) = (2.00, 1.25)` — *recommended default*
  - `(x, y) = (1.50, 1.25)`
  - `(x, y) = (1.50, 1.75)`

- **Failing cases** (these typically make **Test 1 - End Position** fail because the robot's final position won't match within 0.20 m):
  - `(x, y) = (1.50, 2.00)`
  - `(x, y) = (0.50, 0.00)`

> **Procedure** (each time you change the goal):
> ```bash
> cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints
> source install/setup.bash
> colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
> colcon test-result --all
> ```

### 8.2 Create a **timeout failure** (no result received)
In `test/test_waypoints.cpp`, inside **`wait_for_result()`**, you currently have:
```cpp
// auto status = result_future.wait_for(30s);
auto status = result_future.wait_for(60s);
```
To force a timeout error during testing, temporarily **reduce** it to a very small value, e.g.:
```cpp
auto status = result_future.wait_for(1s);
```
Rebuild and run the tests **while the action server is moving slowly or while the goal is far**, so the result cannot arrive within 1 second. This produces a controlled **error** (not a GTest “failure”), matching the grading guide that shows `errors > 0`.

### 8.3 Create a **yaw mismatch failure**
As written, the yaw tolerance is huge:
```cpp
const double per_step = M_PI;
const double tol = 10.0 * per_step;  // ~31.4 rad
```
To make **Test 2 - End Yaw** meaningful and able to fail, **tighten the tolerance** to something realistic, e.g. **10 degrees**:
```cpp
const double per_step = M_PI / 180.0;  // 1 degree in radians
const double tol = 10.0 * per_step;    // 10 degrees
```
Now, choose a passing position pair (e.g., `(2.00, 1.25)`) and run once—you'll likely still pass because the robot faces the goal. To **provoke a yaw failure**, you can:
- Send a goal very close to the robot so minor odometry noise dominates heading, **or**
- Interrupt the motion early (stop the action server or reset the sim), **or**
- Artificially rotate the robot right after it reaches the goal (e.g., with a small script publishing to `/cmd_vel`) before the test samples odometry.

Any of these can make the measured final yaw differ by more than 10°, causing **Test 2** to fail.

> Revert these tolerance changes after you finish demonstrating the failure, unless you want a stricter permanent test.

---

## 9) Full pass and fail walkthroughs

### 9.1 Passing run (both tests pass)
1. Launch the sim:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch fastbot_gazebo one_fastbot_room.launch.py
   ```
2. Run the action server (new terminal):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run fastbot_waypoints fastbot_action_server
   ```
3. In `test/test_waypoints.cpp`, set:
   ```cpp
   double goal_x = 2.00;
   double goal_y = 1.25;
   // keep yaw tol large or set to ~10-20 degrees if you want stricter checks
   ```
4. Build & test:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints
   source install/setup.bash
   colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
   colcon test-result --all
   ```
5. **Expected**: `2 tests, 0 errors, 0 failures, 0 skipped`.

### 9.2 Failing run via **position mismatch**
1. Keep the sim and server running.
2. In `test/test_waypoints.cpp`, choose a **fail** goal, e.g.:
   ```cpp
   double goal_x = 0.50;
   double goal_y = 0.00;
   ```
3. Build & test (same commands as above).
4. **Expected**: The **End Position** test reports a mismatch; summary shows at least one **error** (or failure depending on your exact assertions).

### 9.3 Failing run via **timeout**
1. In `wait_for_result()`, temporarily set:
   ```cpp
   auto status = result_future.wait_for(1s);
   ```
2. Choose a far (but reachable) goal (e.g., `2.00, 1.25`), rebuild & test **while the robot is still traveling**.
3. **Expected**: The test logs a timeout and reports an **error**; summary resembles the grading guide's failing example.

### 9.4 Failing run via **yaw mismatch**
1. Tighten yaw tolerance to **10°** as shown in §8.3.
2. Provoke a heading discrepancy (e.g., disturb the robot's orientation right after arrival).
3. Rebuild & test.
4. **Expected**: **End Yaw** test fails with a message including the measured yaw, desired yaw, and their difference.

---

## 10) Troubleshooting tips

- **No odometry detected**: The test waits for `/fastbot/odom` before sending a goal. If it times out, verify the topic exists:
  ```bash
  ros2 topic echo /fastbot/odom
  ```
- **Action server unavailable**: The test waits up to `10s` for `fastbot_as`. Ensure `fastbot_action_server` is running and advertising the correct action name.
- **Build/test cache**: If results look stale, clean and rebuild:
  ```bash
  rm -rf build/ install/ log/
  colcon build --packages-select fastbot_waypoints
  ```
- **Gazebo stuck**: Kill hanging processes (see §4). Then relaunch.

---

## 11) Grading checklist mapping

- **README present with pass/fail instructions** → ✅ This file.
- **Run sim** → §4.
- **Run action server** → §5.
- **Passing conditions demonstrated** → §9.1 (and §8.1 “success” goals).
- **Failing conditions demonstrated** → §9.2 Position, §9.3 Timeout, §9.4 Yaw.

Good luck—and happy testing!
