# e-Yantra Khojo-Drone (KD) — ROS 2 Workspace

This repository contains the complete ROS 2 workspace for the **e-Yantra Robotics Competition (eYRC 2026-27) Khojo Drone Theme** (Team: **KD_6479**).

---

## 📋 Table of Contents
- [Prerequisites](#-prerequisites)
- [Workspace Setup & Build](#-workspace-setup--build)
- [How to Run](#-how-to-run)
  - [1. Launch Simulation Environment](#1-launch-simulation-environment)
  - [2. Run the PID Controller](#2-run-the-pid-controller)
  - [3. PID Tuning GUI (Optional)](#3-pid-tuning-gui-optional)
  - [4. Run Task 1A (Vision Pipeline)](#4-run-task-1a-vision-pipeline)
  - [5. Replaying Task Bags (Task 1B / 1C)](#5-replaying-task-bags-task-1b--1c)
- [Project Directory Structure](#-project-directory-structure)
- [Troubleshooting](#-troubleshooting)

---

## 🛠 Prerequisites

Ensure your system meets the following requirements:
* **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2 Distribution**: [ROS 2 Humble Hawksbill (Desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* **Build Tools**: `colcon`, `rosdep`
* **Python Libraries**:
  ```bash
  pip3 install opencv-python numpy pyyaml
  ```

---

## 🚀 Workspace Setup & Build

Clone this repository and build the workspace:

```bash
# 1. Clone the repository
git clone git@github.com:AstrolidonSpace/Eyantra-Team-IItm-2026.git pico_ws
cd pico_ws

# 2. Source ROS 2 Humble base installation
source /opt/ros/humble/setup.bash

# 3. Install any missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build all packages
colcon build --symlink-install

# 5. Source the workspace overlay
source install/setup.bash
```

> [!TIP]
> Add `source ~/pico_ws/install/setup.bash` to your `~/.bashrc` to avoid sourcing the overlay in every new terminal.

---

## 🕹 How to Run

Running the simulation requires multiple terminal windows (or tabs).

### 1. Launch Simulation Environment
This brings up the **MuJoCo simulation bridge**, drone model, and the **WhyCode vision tracking node**:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 launch swift_pico swift_pico_simulation.launch.py
```

### 2. Run the PID Controller
Once the simulation and WhyCode nodes are active, launch the position controller:

* **C++ Controller (Default):**
  ```bash
  # Terminal 2
  source /opt/ros/humble/setup.bash
  source ~/pico_ws/install/setup.bash
  ros2 run swift_pico pico_controller_PID
  ```

* **Python Controller:**
  ```bash
  # Terminal 2 (alternative)
  source /opt/ros/humble/setup.bash
  source ~/pico_ws/install/setup.bash
  ros2 run swift_pico pico_controller_PID.py
  ```

The controller subscribes to WhyCode marker feedback (`/whycode_node/markers`), computes PID errors, and publishes flight commands to `/drone_command`.

### 3. PID Tuning GUI (Optional)
To tune Roll, Pitch, and Throttle PID values live via the tuning slider interface:

```bash
# Terminal 3
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 launch pid_tune pid_tune_drone.launch.py
```

### 4. Run Task 1A (Vision Pipeline)
To test ArUco marker detection and arena corner extraction:

```bash
cd ~/pico_ws/src/swift_pico/scripts
python3 KD_6479_task1a.py --image image_1.jpg
```
Results will be output to `image_1_results.txt`.

### 5. Replaying Task Bags (Task 1B / 1C)
To inspect recorded telemetry bags and error profiles:

```bash
# Play Task 1B bag
ros2 bag play task_1b/

# Play Task 1C bag
ros2 bag play task_1c/

# Echo recorded position error in a separate terminal
ros2 topic echo /pos_error
```

---

## 📁 Project Directory Structure

```text
pico_ws/
├── README.md               # Setup and usage documentation
├── .gitignore              # Ignores build/, install/, and log/ artifacts
├── task_1b/                # Task 1B rosbag files and submission zip
├── task_1c/                # Task 1C rosbag files and submission zip
└── src/
    ├── swift_pico/         # Core drone package (PID/LQR controllers, bridge, launch files)
    │   ├── launch/         # swift_pico_simulation.launch.py
    │   ├── scripts/        # Task 1A script, test images
    │   └── src/            # pico_controller_PID.cpp, pid_values.yaml
    ├── controller_tuner/   # PID GUI tuning package (pid_tune)
    ├── rotors_simulator/   # Drone Gazebo simulation models and meshes
    ├── swift_pico_description/ # Robot description, arena assets, URDF/SDF
    └── whycode-ros2/       # WhyCode localization and marker tracking package
```

---

## ❓ Troubleshooting

* **Missing topic or connection issues**:
  Ensure FastDDS profile is properly loaded. The simulation launch automatically sets `FASTRTPS_DEFAULT_PROFILES_FILE` pointing to `swift_pico/config/fastdds_profile.xml`.
* **Build fails with missing messages**:
  Run `colcon build --packages-select swift_msgs controller_msg error_msg whycode_interfaces` first, then run `colcon build`.
* **Shared Memory Error**:
  Ensure permissions allow POSIX shared memory for `/dev/shm` (used by `swift_pico_image_raw`).
