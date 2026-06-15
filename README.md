# UR5 Robotic Arm Pick-and-Place Simulation Workspace

A robust ROS 2 and Gazebo Classic simulation workspace for a UR5 robotic arm equipped with a Robotiq 140 adaptive parallel gripper and a camera. This project implements a fully automated, collision-free two-phase pick-and-place task in the presence of dynamic obstacles using MoveIt 2.

---

## 🚀 Key Features

* **Two-Phase Execution Cycle:**
  * **Phase 1 (Without Obstacle):** Moves from Home $\rightarrow$ Cartesian descent to Cube $\rightarrow$ Grasp $\rightarrow$ Lift $\rightarrow$ Place at Destination.
  * **Phase 2 (With Obstacle):** Spawns a collision obstacle directly in the robot's default path. MoveIt 2 dynamically recalculates a collision-free path around the block to safely place the cube.
* **Gentle Grasping Sequence:** Employs a custom two-stage gripper closing sequence combined with a Gazebo link-attacher plugin to eliminate physics contact jitter ("squirt-out") and guarantee grasping stability.
* **Optimized Kinematics:** Custom kinematic constraints resolved by releasing joint-space limits on the UR5 elbow joint, expanding the planning workspace to safely avoid obstacles without table collision.

---

## 📁 Repository Structure

| Directory / File | Description |
| :--- | :--- |
| `src/ur_yt_sim/` | Core package containing custom simulation worlds, URDF/Xacro descriptions, and execution scripts. |
| `src/ur_yt_sim/src/pick_place.cpp` | C++ Orchestrator node managing MoveIt planning, Gazebo models, and gripper actions. |
| `src/ur5_camera_gripper_moveit_config/` | MoveIt 2 configuration package (SRDF, joint limits, kinematic solver parameters). |
| `src/IFRA_LinkAttacher/` | Gazebo plugin used to spawn virtual rigid joints between the gripper and the cube. |
| `cleanup_gazebo.sh` | Utility shell script to clean up orphaned Gazebo and ROS 2 background processes. |

---

## 🛠️ Installation & Building

### Prerequisites
* **ROS 2 Version:** Humble Hawksbill
* **OS:** Ubuntu 22.04 LTS
* **Dependencies:** Gazebo Classic, MoveIt 2, ROS 2 Controllers.

### Building the Workspace
Clone the repository, navigate to the workspace root, and build using `colcon`:

```bash
cd ~/ur5_ws
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the environment
source install/setup.bash
```

---

## 🎮 Running the Simulation

Follow these steps to launch the simulation and run the pick-and-place task.

### Step 1: Start the Gazebo & MoveIt Environment
Launch the simulation, controllers, planning scene, and RViz:
```bash
ros2 launch ur_yt_sim spawn_ur5_camera_gripper_moveit.launch.py
```

### Step 2: Run the Pick-and-Place Task
Open a new terminal, source the workspace, and start the task node:
```bash
source ~/ur5_ws/install/setup.bash
ros2 launch ur_yt_sim run_pick_place.launch.py
```

### Step 3: Cleanup (Between runs)
If Gazebo processes lock up or crash, run the helper script in the workspace root:
```bash
./cleanup_gazebo.sh
```


---

## 📹 Demo Videos

### Phase 1: Without Obstacle
<video src="https://github.com/KevinBorda07/ur5_pick_place_optimized_path/raw/main/src/Output/without_obstacle.mp4" width="700" controls></video>

### Phase 2: With Obstacle (Dynamic Obstacle Avoidance)
<video src="https://github.com/KevinBorda07/ur5_pick_place_optimized_path/raw/main/src/Output/with_obstacle.mp4" width="700" controls></video>





---

## ⚙️ Technical Details


> [!NOTE]
> **Mimic Joint Warnings:**
> During startup, you will see MoveIt log messages stating `Joint '..._mimic' not found in model 'ur'`. These are **non-fatal warnings** and can be safely ignored. They occur because the Gazebo ROS 2 control plugin registers internal virtual joints to handle simulation physics coupling, which are not defined in the robot's planning URDF.

> [!IMPORTANT]
> **Grasping Optimization:**
> The physical grasping is implemented in two steps in `pick_place.cpp`:
> 1. The gripper closes to a pre-contact position ($0.46$ rad).
> 2. The `/ATTACHLINK` service is called to create a rigid fixed joint between the gripper and the cube.
> 3. The gripper closes completely to a contact position ($0.52$ rad) under low effort.
> This sequence prevents the object from slipping or flying away due to contact forces in Gazebo.
