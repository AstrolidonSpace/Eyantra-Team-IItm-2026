# e-Yantra Khoj-o-Drone (KD) — ROS 2 Workspace

This repository contains the complete ROS 2 workspace for the **e-Yantra Robotics Competition (eYRC 2026-27) Khoj-o-Drone Theme** (Team: **KD_6479**).

---

## 📋 Table of Contents
- [Prerequisites](#-prerequisites)
- [Workspace Setup & Build](#-workspace-setup--build)
- [Interactive Architecture & Flight Simulator Web App](#-interactive-architecture--flight-simulator-web-app)
- [How to Run](#-how-to-run)
  - [1. Launch Simulation Environment](#1-launch-simulation-environment)
  - [2. Task 2A: Position Hold PID Controller](#2-task-2a-position-hold-pid-controller)
  - [3. Task 2B: Autonomous Waypoint Navigation (Actions & Services)](#3-task-2b-autonomous-waypoint-navigation-actions--services)
  - [4. PID Tuning GUI (Optional)](#4-pid-tuning-gui-optional)
  - [5. Run Task 1A (Vision Pipeline)](#5-run-task-1a-vision-pipeline)
  - [6. Replaying Task Bags (Task 1B / 1C / 2A / 2B)](#6-replaying-task-bags-task-1b--1c--2a--2b)
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

# 4. Build all packages (including waypoint_navigation and swift_pico)
colcon build --symlink-install

# 5. Source the workspace overlay
source install/setup.bash
```

> [!TIP]
> Add `source ~/pico_ws/install/setup.bash` to your `~/.bashrc` to avoid sourcing the overlay in every new terminal.

---

## 🌐 Interactive Architecture & Flight Simulator Web App

An interactive web application and live 3D quadrotor flight simulator is available in [`docs/`](docs/) to visualize and experiment with [`pico_controller_PID.cpp`](src/swift_pico/src/pico_controller_PID.cpp).

### Launching the Web App Locally:
```bash
# Run local HTTP server
python3 -m http.server 8080 --directory docs

# Open in browser:
xdg-open http://localhost:8080/
```

### What's Included:
* **Interactive 3D Drone Flight Arena**: Drag and drop waypoints in real-time, observe simulated physics, and test wind gust disturbance recovery.
* **Cascaded Altitude Visualizer**: Live demonstration showing why dual-loop velocity damping prevents vertical oscillations.
* **Live Oscilloscope `/pos_error` Chart**: Real-time error plotting at 30.3 Hz.
* **Live Actuator Telemetry**: RC PWM meters (`1000–2000 µs`) with neutral 1500 centering.
* **Interactive Code Inspector**: Line-by-line annotations of the production C++ implementation.

> [!NOTE]
> See [`docs/README.md`](docs/README.md) for detailed documentation and free **GitHub Pages** deployment instructions.

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

---

### 2. Task 2A: Position Hold PID Controller
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

The controller subscribes to WhyCode marker feedback (`/whycode_node/markers`), computes cascaded dual-loop PID errors (position-to-velocity, velocity-to-angle/thrust), and publishes flight commands to `/drone_command`.

---

### 3. Task 2B: Autonomous Waypoint Navigation (Actions & Services)

Task 2B implements a complete rescue route sequence navigating through four 3D waypoints (`wp1` $\rightarrow$ `wp4`) using ROS 2 Actions and Services:
* **Waypoint Service** (`GetWaypoints.srv`): Serves the predefined waypoints in exact sequence.
* **Action Client** (`pico_client`): Fetches waypoints and sends them one by one as goals to the action server.
* **Action Server** (`pico_server`): Hosts `NavToWaypoint.action`, controls flight via cascaded PID, streams real-time pose and hold-time feedback, stabilizes within $\pm 0.4$ units across all axes for $\ge 3$ consecutive seconds, and returns total hover settling time.

Run the nodes in separate terminals in the following sequence:

#### Step 1: Launch Simulator
```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 launch swift_pico swift_pico_simulation.launch.py
```

#### Step 2: Run Waypoint Service
```bash
# Terminal 2 (C++)
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 run swift_pico waypoint_service_cpp

# Or Python:
# ros2 run swift_pico waypoint_service
```

#### Step 3: Run Action Client (waits for server)
```bash
# Terminal 3 (C++)
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 run swift_pico pico_client_cpp

# Or Python:
# ros2 run swift_pico pico_client
```

#### Step 4: Run Action Server (starts flight navigation)
```bash
# Terminal 4 (C++)
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 run swift_pico pico_server_cpp

# Or Python:
# ros2 run swift_pico pico_server
```

---

### 4. PID Tuning GUI (Optional)
To tune Roll, Pitch, and Throttle PID values live via the tuning slider interface:

```bash
# Terminal 3 (or separate)
source /opt/ros/humble/setup.bash
source ~/pico_ws/install/setup.bash
ros2 launch pid_tune pid_tune_drone.launch.py
```

---

### 5. Run Task 1A (Vision Pipeline)
To test ArUco marker detection and arena corner extraction:

```bash
cd ~/pico_ws/src/swift_pico/scripts
python3 KD_6479_task1a.py --image image_1.jpg
```
Results will be output to `image_1_results.txt`.

---

### 6. Replaying Task Bags (Task 1B / 1C / 2A / 2B)
To inspect recorded telemetry bags and error profiles:

```bash
# Play Task 1B bag
ros2 bag play task_1b/

# Play Task 1C bag
ros2 bag play task_1c/

# Play Task 2A bag
ros2 bag play task_2a/

# Play Task 2B bag
ros2 bag play task_2b/

# Echo recorded position error in a separate terminal
ros2 topic echo /pos_error
```

---

## 📁 Project Directory Structure

```text
pico_ws/
├── README.md               # Setup, architecture, and usage documentation
├── .gitignore              # Ignores build/, install/, and log/ artifacts
├── docs/                   # Interactive architecture guide & web flight simulator
│   ├── index.html          # Web application structure
│   ├── style.css           # Modern aerospace dark theme styling
│   ├── app.js              # Real-time physics engine, PID math & canvas renderer
│   └── README.md           # Documentation & GitHub Pages deployment guide
├── task_1b/                # Task 1B rosbag files and submission zip
├── task_1c/                # Task 1C rosbag files and submission zip
├── task_2a/                # Task 2A rosbag files and submission C++ controller
├── task_2b/                # Task 2B rosbag files and submission C++ nodes
└── src/
    ├── swift_pico/         # Core drone package (controllers, bridge, launch files)
    │   ├── launch/         # swift_pico_simulation.launch.py
    │   ├── scripts/        # Task 1A script, test images, grid coordinates JSON
    │   └── src/            # pico_controller_PID.cpp, pico_server.cpp, pico_client.cpp, waypoint_service.cpp
    ├── waypoint_navigation/ # Custom ROS 2 interfaces package
    │   ├── action/         # NavToWaypoint.action (goal, result, feedback)
    │   └── srv/            # GetWaypoints.srv (service definition)
    ├── controller_tuner/   # PID GUI tuning package (pid_tune)
    ├── rotors_simulator/   # Drone Gazebo simulation models, interface and meshes
    ├── swift_pico_description/ # Robot description, arena assets, URDF/SDF
    └── whycode-ros2/       # WhyCode localization and marker tracking package
```

---

## ❓ Troubleshooting

* **Missing topic or connection issues**:
  Ensure FastDDS profile is properly loaded. The simulation launch automatically sets `FASTRTPS_DEFAULT_PROFILES_FILE` pointing to `swift_pico/config/fastdds_profile.xml`.
* **Build fails with missing messages or interfaces**:
  Run `colcon build --packages-select waypoint_navigation swift_msgs controller_msg error_msg whycode_interfaces` first, then run `colcon build`.
* **Shared Memory Error**:
  Ensure permissions allow POSIX shared memory for `/dev/shm` (used by `swift_pico_image_raw`).
* **Yaw Drift / Bang-Bang Warning**:
  The simulation bridge enforces `max.rate_yaw = ±45°/s` for any non-neutral command. Keep `rc_yaw = 1500` to rigidly command `yaw_rate = 0.0 rad/s` and maintain frame alignment.
