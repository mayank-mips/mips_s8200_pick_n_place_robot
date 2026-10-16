# mips_s8200_pick_n_place_robot

AI-driven **pick-and-place** demo for a Fanuc LR-Mate 200iD-7L arm in **Gazebo (ROS 1 Noetic)**:

1. A **depth camera** senses the workspace.
2. An **AI/vision node** detects the target object's 3D location.
3. An **inverse-kinematics** motion node (MoveIt) moves the arm to the object.
4. A **vacuum/attach grasp plugin** picks the object up.
5. A **state machine** moves it to a drop location and releases it.

The base robot model (Fanuc arm, meshes, joint controllers) comes from
[intelligent-control-lab/Robot_Digital_Twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin)
(MIT licensed) — credit to the original authors.

---

## Architecture / data flow

```
┌──────────────────────────────────────────────┐
│ Gazebo                                         │
│   • Fanuc arm (+ suction tip)                  │
│   • Depth camera (eye-to-hand, top-down)       │
│   • Table + red target box + drop marker       │
│   • gazebo_grasp_plugin (attach on contact)    │
└───┬───────────────────────────────▲───────────┘
    │ /camera/color/image_raw        │ /fanuc_gazebo/jointN_position_controller/command
    │ /camera/depth/points           │ (via MoveIt)
    ▼                                │
┌─────────────────┐  /detected_object_pose   ┌──────────────────────┐
│ object_detector │ ───────────────────────▶ │ pick_and_place_fsm   │
│ (HSV blob → 3D) │                           │  detect→approach→    │
└─────────────────┘                           │  grasp→move→release  │
                                              └─────┬──────┬─────────┘
                                   /motion_goal     │      │  /vacuum/grasp
                                                    ▼      ▼
                                           ┌────────────┐ ┌──────────────┐
                                           │ motion_ik  │ │ vacuum_gripper│
                                           │ (MoveIt/KDL)│ │ (SetBool)    │
                                           └────────────┘ └──────────────┘
```

---

## Requirements

- **Ubuntu 20.04 + ROS Noetic** (e.g. the `osrf/ros:noetic-desktop-full` Docker image on WSL2).
- Gazebo classic (bundled with `desktop-full`), MoveIt, OpenCV/cv_bridge, PCL.

> ROS Noetic only exists for Ubuntu 20.04. On newer Ubuntu use the Docker image.

---

## Setup (Docker image `osrf/ros:noetic-desktop-full` on WSL2)

```bash
# --- inside the container ---
source /opt/ros/noetic/setup.bash
apt update && apt install -y git python3-catkin-tools

# 1. Create a catkin workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 2. Clone the base robot model + this package
git clone https://github.com/intelligent-control-lab/Robot_Digital_Twin.git
git clone https://github.com/mayank-mips/mips_s8200_pick_n_place_robot.git

# 3. Clone the vacuum grasp plugin (NOT in apt) and its deps
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
git clone https://github.com/JenniferBuehler/general-message-pkgs.git

# 4. Install extra ROS dependencies
apt install -y \
  ros-noetic-moveit \
  ros-noetic-trac-ik-kinematics-plugin \
  ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-gazebo-ros-control \
  ros-noetic-pcl-ros python3-opencv

# 5. Build
cd ~/catkin_ws
catkin_make        # or: catkin build
source devel/setup.bash
```

> **Important:** the arm xacro in `urdf/fanuc_with_camera.xacro` includes
> `robot_digital_twin/urdf/fanuc_arm.xacro` and references link names
> `base_link` and `link_6`. Open that file and adjust the link names and the
> camera mount pose to match the actual Fanuc model if they differ.

---

## Run the full demo

```bash
roslaunch mips_pick_n_place pick_and_place_demo.launch
```

Gazebo opens with the arm, depth camera, table, and a red box. The pipeline then
runs end to end: detect → approach → descend → suction on → lift → move to drop
→ descend → release → retreat.

### Run components independently (for debugging)

```bash
# 1. Simulation + arm + camera + controllers
roslaunch mips_pick_n_place bringup.launch

# 2. Perception only (watch the detected pose)
roslaunch mips_pick_n_place perception.launch
rostopic echo /detected_object_pose
rqt_image_view            # view /camera/color/image_raw

# 3. Motion + gripper only
roslaunch mips_pick_n_place motion.launch use_moveit:=true

# 4. Then start the FSM
rosrun mips_pick_n_place pick_and_place_fsm.py
```

---

## Topic / service interface

| Purpose | Name | Type |
|---|---|---|
| Color image (sensor) | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Depth point cloud (sensor) | `/camera/depth/points` | `sensor_msgs/PointCloud2` |
| Detected object pose | `/detected_object_pose` | `geometry_msgs/PoseStamped` |
| Motion goal (to IK node) | `/motion_goal` | `geometry_msgs/PoseStamped` |
| Joint commands (actuators) | `/fanuc_gazebo/jointN_position_controller/command` | `std_msgs/Float64` |
| Joint feedback | `/fanuc_gazebo/joint_states` | `sensor_msgs/JointState` |
| Vacuum grasp on/off | `/vacuum/grasp` | `std_srvs/SetBool` |

---

## Plugging in your own AI

**1. Perception — replace the color detector.**
In `scripts/object_detector.py`, the `ColorBlobDetector` class is isolated behind
a single contract:

```python
detect(bgr_image: np.ndarray) -> (u, v) | None   # object center pixel
```

Drop in a trained model (e.g. YOLO): run inference on `bgr_image`, take the
best detection, and return its center pixel `(u, v)`. Everything downstream
(depth lookup from the point cloud, TF to base frame, publishing
`/detected_object_pose`) stays unchanged.

**2. Motion — replace or augment the policy.**
`scripts/motion_ik.py` maps a target pose → joint motion via MoveIt. To use a
learned motion policy instead, publish joint targets directly to the
`jointN_position_controller/command` topics (bypassing MoveIt), or wrap this
read/command interface in a Gymnasium `Env` for reinforcement learning
(`reset()` → respawn object, `step(action)` → publish commands, observe
`/joint_states` + camera, compute reward).

**3. Tuning.** Object color (`config/color_detector.yaml`), pick/drop poses and
offsets (`config/pick_and_place.yaml`), and controller PID gains
(`config/robot_control_6dof.yaml`) are all editable without code changes.

---

## Notes & manual steps

- **Grasp plugin:** `gazebo_grasp_plugin` (from `JenniferBuehler/gazebo-pkgs`)
  is built from source (step 3 above), not apt. It attaches a contacting object
  rigidly to the suction tip and releases on separation. The plugin arm config
  lives in `urdf/suction_tip.xacro` — adjust `palm_link`/`gripper_link` to your
  arm's flange link name.
- **MoveIt config:** this scaffold calls MoveIt with `move_group=manipulator`.
  Generate a MoveIt config for the Fanuc arm with the MoveIt Setup Assistant
  (define the `manipulator` planning group and `suction_tip` end effector), or
  switch to the KDL fallback with `use_moveit:=false` and implement `solve_ik()`
  in `motion_ik.py` (e.g. with `trac_ik_python`).
- **GUI on WSL:** run with Windows 11 WSLg (or an X server). Gazebo can be slow
  without GPU passthrough; that's expected.

## License

MIT. Base robot model from
[intelligent-control-lab/Robot_Digital_Twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin) (MIT).
