# 🌐 Interactive Architecture & Flight Simulator Web Documentation

This directory contains the interactive web documentation and live flight simulator for the Swift Pico closed-loop controller ([`pico_controller_PID.cpp`](../src/swift_pico/src/pico_controller_PID.cpp)).

---

## 🚀 How to Launch Locally

You can run this webpage locally using Python's built-in HTTP server:

```bash
# Navigate to the workspace root
cd ~/pico_ws

# Launch the server on port 8080
python3 -m http.server 8080 --directory docs
```

Now open your browser and navigate to:
👉 **[http://localhost:8080/](http://localhost:8080/)**

To launch it directly from the terminal on Ubuntu:
```bash
xdg-open http://localhost:8080/
```

---

## 🛰 How to Enable Free GitHub Pages Hosting

You can host this interactive simulator publicly or for your teammates directly on GitHub Pages:

1. Open your repository on GitHub: [Eyantra-Team-IItm-2026](https://github.com/AstrolidonSpace/Eyantra-Team-IItm-2026).
2. Go to **Settings** &rarr; **Pages** (under Code and automation).
3. Under **Build and deployment**:
   * **Source**: `Deploy from a branch`
   * **Branch**: `main`
   * **Folder**: `/docs`
4. Click **Save**.
5. Your interactive documentation will be live at:
   `https://astrolidonspace.github.io/Eyantra-Team-IItm-2026/`

---

## 🎨 Features & Capabilities

### 1. Live 3D Flight Arena & Physics Simulator
* **Interactive Canvas**: Renders the 3D isometric arena, ground shadow projection, rotating quadrotor propellers, and WhyCode tracking marker.
* **Drag & Drop Setpoints**: Click and drag anywhere inside the arena to move the target waypoint dynamically.
* **Flight Dynamics**: Implements Euler-integrated quadrotor equations responding to RC PWM throttle, pitch, roll, drag, and gravity.
* **Presets & Wind Impulses**: Includes preset waypoints (`Default [1.5, -1.5, 13.0]`, `Hover Center [0, 0, 10.0]`, `Corner Step [-2, 2, 16.0]`) and a **Simulate Wind Gust** button to test PID disturbance rejection.

### 2. Live Actuator Telemetry & Oscilloscope
* **RC PWM Meter Gauges**: Real-time visualization of `rc_roll`, `rc_pitch`, and `rc_throttle` centered around 1500 µs neutral, bounded by `[1000, 2000]` safety limits.
* **Live `/pos_error` Rolling Chart**: Continuous 30 Hz stream plotting Roll (Cyan), Pitch (Purple), and Altitude (Emerald) tracking error over time.
* **Velocity Metrics**: Displays outer-loop target vertical velocity ($v_{z,\text{des}}$), measured vertical speed ($v_{z,\text{act}}$), and velocity error ($e_{v_z}$).

### 3. Real-Time PID Tuning Sliders
* Dynamic adjustment of $K_p$, $K_i$, and $K_d$ for Roll, Pitch, and the Cascaded Altitude Outer Loop.
* Instant visual feedback on drone stability and overshoot behavior without restarting ROS 2 nodes.

### 4. Step-by-Step Signal Pipeline
Interactive breakdown of the 5 core stages in `pico_controller_PID.cpp`:
1. **WhyCode Perception**: Overhead camera marker tracking.
2. **Error Computation**: Spatial tracking error vectors.
3. **Lateral Roll & Pitch PID**: Independent parallel loops with anti-windup clamping ($\pm 40.0$).
4. **Cascaded Altitude Architecture**: Height Error &rarr; Desired Velocity ($\pm 3.0$ m/s) &rarr; Inner Throttle PWM ($K_{p,\text{vel}} = 15.0$).
5. **RC Mapping & Arming**: 1500 neutral offset, sign inversion, and `rc_aux4 = 2000` arm flag.

### 5. Interactive Source Code Inspector
Tabbed, syntax-highlighted code explorer matching production lines in [`src/swift_pico/src/pico_controller_PID.cpp`](../src/swift_pico/src/pico_controller_PID.cpp).

---

## 🧭 Task 2C: Interactive Path Planner & Vision Perception (`planner.html`)

Access the dedicated Task 2C web app at:
👉 **[http://localhost:8080/planner.html](http://localhost:8080/planner.html)**

### What's Included in the Task 2C Explorer:
* **Interactive $11 \times 11$ Disaster Arena Canvas**: Click any intersection to add/relocate survivors, test benchmark layouts, and observe the live drone flight with 3-second hold timers.
* **Perception Pipeline Step-by-Step**: Interactive visual breakdown of ArUco boundary detection, perspective warping to $900 \times 900$, dual-range HSV red masking, and spatial moment grid snapping.
* **Algorithm Showdown: TSP vs Greedy Nearest Neighbor**: Clear comparison illustrating why greedy nearest-neighbor traps the drone into crossing the arena diagonally (20.94 m vs 19.38 m), and why combinatorial brute-force TSP is globally optimal.
* **Live 24-Route Permutation Explorer**: Searchable and filterable table of all $4! = 24$ candidate routes with distance deltas, leg breakdowns, and real-time canvas route preview on click.
* **Metric Coordinate Table**: Dynamic WhyCode $(X, Y)$ lookups from `grid_whycode_coordinates.csv`.

---

## 📂 File Structure

```text
docs/
├── index.html                   # Task 2A: Flight Controller Simulator & Architecture Guide
├── planner.html                 # Task 2C: Vision Pipeline & Optimal TSP Route Planner Explorer
├── style.css                    # Unified aerospace dark mode design system & glassmorphism
├── app.js                       # Task 2A physics engine, PID math, canvas renderer & UI sync
├── planner.js                   # Task 2C 11x11 arena canvas, TSP solver, flight animator & HUD
├── grid_whycode_coordinates.json# Metric WhyCode (X, Y) coordinate lookup table
├── task2c_route.png             # Checkpoint route image from Task 2C simulation
└── README.md                    # Documentation and hosting guide (this file)
```
