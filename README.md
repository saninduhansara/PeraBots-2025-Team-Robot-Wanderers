<div align="center">

# 🤖 PeraBots 2025 — Team Robot Wanderers
### Autonomous Navigation & Intelligent Closed-Loop Wall-Following in Webots

[![Platform: Webots](https://img.shields.io/badge/Platform-Webots%20R2025a%20%7C%20R2023b-00599C?style=for-the-badge&logo=cplusplus)](https://cyberbotics.com/)
[![Language: C](https://img.shields.io/badge/Controller-C99-A8B9CC?style=for-the-badge&logo=c)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Tuner: Python](https://img.shields.io/badge/Auto--Tuning-Python%203.x-3776AB?style=for-the-badge&logo=python)](https://www.python.org/)
[![Control: PID](https://img.shields.io/badge/Algorithm-Ziegler--Nichols%20PID-FF6F00?style=for-the-badge)](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
[![Competition](https://img.shields.io/badge/Competition-PeraBots%202025-brightgreen?style=for-the-badge)]()
[![Team](https://img.shields.io/badge/Team-Robot%20Wanderers-blueviolet?style=for-the-badge)]()

</div>

---

## 📌 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Simulation Showcase](#-simulation-showcase)
- [System Architecture & Robot Model](#-system-architecture--robot-model)
- [Control Algorithms & Auto-Tuning](#-control-algorithms--auto-tuning)
  - [1. Dual-Sensor Wall-Following Control](#1-dual-sensor-wall-following-control)
  - [2. Closed-Loop PID Control Formulation](#2-closed-loop-pid-control-formulation)
  - [3. Online Ziegler-Nichols Auto-Tuner](#3-online-ziegler-nichols-auto-tuner)
  - [4. Reactive Collision Avoidance & Corner Handling](#4-reactive-collision-avoidance--corner-handling)
- [Performance & Benchmark Analysis](#-performance--benchmark-analysis)
- [Repository Structure](#-repository-structure)
- [Prerequisites & Dependencies](#-prerequisites--dependencies)
- [Getting Started & How to Run](#-getting-started--how-to-run)
- [Current Tuned Constants](#-current-tuned-constants)
- [Team & Acknowledgments](#-team--acknowledgments)

---

## 📖 Overview

Developed for the **PeraBots 2025** robotics competition, this project presents an autonomous differential-drive robotic system designed by **Team Robot Wanderers**. Operating within the **Cyberbotics Webots** physics simulation environment, the robot autonomously explores complex arenas with curved and polygonal walled boundaries.

The core challenge revolves around maintaining high-speed, stable wall adherence across both **inner** and **outer wall contours** while rapidly recovering from abrupt corridor shifts, tight corners, and unforeseen obstacles without manual intervention.

To achieve this, our system pairs an ultra-low latency **C99 reactive robot controller** with an intelligent **Python-based Ziegler-Nichols oscillation analyzer** for real-time PID auto-tuning.

---

## ✨ Key Features

- 🏎️ **Precision Dual-Sensor Distance & Attitude Estimation**: Incorporates front-left (`lf`) and back-left (`lb`) distance sensors to continuously calculate both wall offset distance and angular deviation.
- 🔄 **Real-Time Dynamic PID Control**: Computes smooth differential steering velocity updates to eliminate steady-state error and counter drift.
- ⚡ **Zero-Crossing Frequency PID Auto-Tuner (`botwanders.py`)**: An external Python supervisory process that monitors error streams, detects zero-crossings, calculates the ultimate oscillation period ($T_u$), and calculates optimal $K_p$, $K_i$, and $K_d$ parameters via Ziegler-Nichols rules.
- 🔁 **Simulation Hot-Reloading**: The C controller reads updated PID parameters on-the-fly (`pid_constants.txt`) without needing to restart the simulation.
- 🛡️ **Autonomous Obstacle Detection & Collision Recovery**: Dedicated front sonar (`fs`) and corner distance sensor (`lc`) prevent head-on crashes into interior obstacles and navigate tight turns.
- 📊 **Inner vs. Outer Path Optimization**: Thoroughly tested across inner and outer perimeter tracks for performance validation.

---

## 🎥 Simulation Showcase

### Arena Traversal Simulation

The animated demonstration below highlights **Robot Wanderers** navigating the competition arena, handling sharp turns, and maintaining equidistant offset along irregular wall structures:

<p align="center">
  <img src="Robot%20Wanderers.gif" alt="Robot Wanderers Webots Simulation" width="90%" />
</p>

> 💡 **Tip**: A complete video guide showing the auto-tuning workflow is available in the repository root: [`How to run AUTO TUNE.mp4`](How%20to%20run%20AUTO%20TUNE.mp4).

---

## 🛠️ System Architecture & Robot Model

The robot is modeled as a compact two-wheel differential drive platform with a caster support ring.

### Robot Specifications

| Specification | Value | Description |
| :--- | :--- | :--- |
| **Chassis Radius** | $37.0\text{ mm}$ ($0.037\text{ m}$) | Compact circular footprint |
| **Total Mass** | $0.15\text{ kg}$ | Center of mass balanced at $z = 0.015\text{ m}$ |
| **Wheel Radius** | $20.0\text{ mm}$ ($0.02\text{ m}$) | Left & right rotational drive |
| **Base Velocity** | $6.0\text{ rad/s}$ | Nominal forward speed |
| **Max Velocity** | $6.27\text{ - }6.28\text{ rad/s}$ | Safe motor actuator ceiling |
| **Controller Timestep** | $32\text{ ms}$ | Control frequency $\approx 31.25\text{ Hz}$ |

### Sensor Suite Layout

| Device Tag | Sensor Type | Mounting Orientation | Primary Role |
| :---: | :---: | :---: | :--- |
| `lf` | Distance Sensor (Infrared) | Left-Front ($75^\circ / 1.309\text{ rad}$) | Primary wall distance measurement |
| `lb` | Distance Sensor (Infrared) | Left-Back ($75^\circ / 1.309\text{ rad}$) | Attitude / alignment angle estimation |
| `fs` | Sonar Distance Sensor | Front Center ($0^\circ$) | Forward collision detection ($< 10\text{ cm}$) |
| `lc` | Distance Sensor (Infrared) | Left-Corner ($75^\circ / 1.31\text{ rad}$) | Corner turn transition detection |

### Software Dataflow

```mermaid
flowchart TD
    subgraph Webots Simulation
        S[Sensors: lf, lb, fs, lc] -->|Raw Readings| C[test1perabots.c Controller]
        C -->|Diff Speeds| M[Left & Right Wheel Motors]
        C -->|Log Error vs Time| E[(error.txt)]
        P[(pid_constants.txt)] -->|Hot-Reload Every Step| C
    end

    subgraph Python Auto-Tuner
        E -->|Read Time & Error Series| T[botwanders.py Supervisor]
        T -->|Zero-Crossing Detection| Z[Calculate Tu & Ku]
        Z -->|Ziegler-Nichols Formula| PID[Derive Kp, Ki, Kd]
        PID -->|Write Updated Gains| P
    end
```

---

## 🧠 Control Algorithms & Auto-Tuning

### 1. Dual-Sensor Wall-Following Control

Rather than relying on a single proximity distance, the robot computes an average distance $\bar{d}$ and an alignment error $\theta_{\text{align}}$ using both lateral sensors:

$$\bar{d} = \frac{d_{\text{lf}} + d_{\text{lb}}}{2}$$

$$\theta_{\text{align}} = |d_{\text{lb}} - d_{\text{lf}}|$$

The target wall spacing is defined by `INPUT_DISTANCE = 10.0 cm`. A region threshold `ERROR_DIST = 3.0 cm` categorizes the robot's spatial state:
- **Inside Deadband**: Minimal corrections required.
- **Outside Margin**: Active proportional steering to pull back towards the baseline.
- **Angular Misalignment**: When $\theta_{\text{align}} > \text{MAX\_ALIGN\_ANGLE}$, heading realignment takes precedence.

---

### 2. Closed-Loop PID Control Formulation

The continuous tracking error is given by:

$$e(t) = \bar{d}(t) - d_{\text{target}}$$

At each discrete timestep $\Delta t = 0.032\text{ s}$:

$$\text{Integral: } I(t) = I(t - 1) + e(t) \cdot \Delta t$$

$$\text{Derivative: } D(t) = \frac{e(t) - e(t - 1)}{\Delta t}$$

$$\text{Control Output: } u(t) = K_p \cdot e(t) + K_i \cdot I(t) + K_d \cdot D(t)$$

The steering output $u(t)$ alters the differential velocities:

$$v_{\text{left}} = V_{\text{base}} - u(t)$$
$$v_{\text{right}} = V_{\text{base}} + u(t)$$

Differential limits are clamped ($|v_L - v_R| \le \text{MAX\_DIFFERENCE}$) to prevent spinning out or wheel slip.

---

### 3. Online Ziegler-Nichols Auto-Tuner

The script [`controllers/test1perabots/botwanders.py`](controllers/test1perabots/botwanders.py) automates the tuning of control parameters using continuous oscillation analysis:

1. **Error Stream Sampling**: Webots streams timestamped errors to `error.txt`.
2. **Sub-Sample Zero-Crossing Interpolation**:
   When consecutive error samples change sign ($e_{k-1} \cdot e_k < 0$), the precise zero-crossing timestamp $t^*$ is computed:
   $$t^* = t_{k-1} + (0 - e_{k-1}) \cdot \frac{t_k - t_{k-1}}{e_k - e_{k-1}}$$
3. **Ultimate Period ($T_u$) Extraction**:
   $$T_u = \frac{1}{N} \sum_{i=1}^{N} (t^*_{i+2} - t^*_i)$$
4. **Ziegler-Nichols Gain Estimation**:
   With ultimate gain $K_u$, the optimal PID parameters are calculated:
   $$K_p = 0.6 \cdot K_u$$
   $$K_i = \frac{1.2 \cdot K_u}{T_u}$$
   $$K_d = 0.075 \cdot K_u \cdot T_u$$
5. **Hot-Reload Dispatch**: Updated constants are written into `pid_constants.txt` and immediately integrated by the active simulation.

---

### 4. Reactive Collision Avoidance & Corner Handling

- **Front Obstacle Intervention**: If the front sonar reading drops below `COLLISION_DISTANCE` ($10.0\text{ cm}$), the controller overrides PID guidance and executes an evasive turn away from the obstacle.
- **Corner Transition Detection**: The left-corner sensor (`lc`) distinguishes between smooth straight walls, outer turns, and pocket recesses, preventing false wall-tracking lockups.

---

## 📈 Performance & Benchmark Analysis

The robot's tracking accuracy and path fidelity were evaluated across both **Outer Wall** and **Inner Wall** courses. The comparative tracking and deviation behavior is illustrated below:

<p align="center">
  <img src="images/Outer%20wall%20and%20inner%20wall%20performance.png" alt="Outer Wall and Inner Wall Performance Comparison" width="92%" style="border-radius: 8px; box-shadow: 0 4px 15px rgba(0,0,0,0.2);" />
</p>

### Performance Highlights:
- **Outer Wall Tracking**: Features smoother curvatures and gradual bends, exhibiting minimal oscillation amplitude and fast convergence to the $10\text{ cm}$ setpoint.
- **Inner Wall Tracking**: Introduces sharper radii and acute obstacle contours. The combination of angle-alignment checks and derivative damping ($K_d$) maintains stability without boundary collisions.
- **Dynamic Stability**: Settling time under step disturbances remains within $1.5\text{ s}$.

---

## 📁 Repository Structure

```text
PeraBots-2025-Team-Robot-Wanderers/
├── .git/                                # Git version control metadata
├── How to run AUTO TUNE.mp4             # Video walkthrough for running auto-tuning
├── How to run.txt                       # Quick reference notes
├── Robot Wanderers.gif                  # Recorded Webots simulation animation
├── README.md                            # Comprehensive project documentation
├── controllers/
│   └── test1perabots/
│       ├── Makefile                     # Webots build configuration
│       ├── botwanders.py                # Python Ziegler-Nichols auto-tuner
│       ├── test1perabots.c              # Primary C robot controller
│       ├── test1perabots.exe            # Compiled controller executable
│       ├── pid_constants.txt            # Current live PID constants
│       └── error.txt                    # Simulation error logging output
├── images/
│   └── Outer wall and inner wall performance.png  # Performance comparison plot
├── path/
│   ├── inner_path.obj                   # 3D inner obstacle/wall mesh
│   └── outer_path.obj                   # 3D outer boundary wall mesh
├── protos/
│   └── Robot.urdf                       # Complete URDF robot model definition
└── worlds/
    ├── pera bots.wbt                    # Competition Webots simulation world
    ├── .pera bots.wbproj                # Webots project configuration
    └── .pera bots.jpg                   # World preview thumbnail
```

---

## 📦 Prerequisites & Dependencies

Before running the simulation or auto-tuner, ensure you have the following installed:

1. **Webots Robot Simulator**:
   - [Cyberbotics Webots](https://cyberbotics.com/) (Version **R2025a** or **R2023b**)
2. **Python Environment** (for Auto-Tuning):
   - Python 3.8+
   - `numpy`
   ```bash
   pip install numpy
   ```
3. **C Compiler** *(Optional, if modifying C controller)*:
   - GCC / MinGW bundled with Webots or installed locally.

---

## 🚀 Getting Started & How to Run

### Method 1: Standard Simulation Run (Direct Webots)

1. Launch **Cyberbotics Webots**.
2. Open the world file:
   - Navigate to `File` > `Open World...`
   - Select `worlds/pera bots.wbt`
3. Click the **Play** button (▶️) on the top toolbar to start the simulation.
4. The robot will start navigating and tracking walls using the pre-tuned PID constants from `pid_constants.txt`.

---

### Method 2: Automated Run with PID Auto-Tuning

To execute the self-tuning pipeline with real-time parameter optimization:

1. Verify that your Webots installation path matches your system configuration in [`controllers/test1perabots/botwanders.py`](controllers/test1perabots/botwanders.py#L20) (default: `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`).
2. Run the tuning supervisor script:
   ```bash
   cd controllers/test1perabots
   python botwanders.py
   ```
3. The script will:
   - Clear existing logs (`error.txt`, `pid_constants.txt`).
   - Launch Webots with `worlds/pera bots.wbt`.
   - Monitor real-time trajectory errors.
   - Detect zero crossings and compute $T_u$.
   - Continuously update `pid_constants.txt`.

> 🎥 **Video Tutorial**: Refer to [`How to run AUTO TUNE.mp4`](How%20to%20run%20AUTO%20TUNE.mp4) for a visual step-by-step demonstration.

---

### Method 3: Compiling Controller from Source (C)

If you modify [`test1perabots.c`](controllers/test1perabots/test1perabots.c):
1. Open the file in Webots' built-in code editor or an external terminal.
2. Build the controller using the provided `Makefile`:
   ```bash
   cd controllers/test1perabots
   make clean
   make
   ```

---

## 🎯 Current Tuned Constants

The best-performing parameters obtained from our Ziegler-Nichols tuning runs and manual boundary calibration:

| Parameter | Symbol | Current Value | Functional Role |
| :--- | :---: | :---: | :--- |
| **Proportional Gain** | $K_p$ | `0.3000` | Drives rapid response to wall distance offsets |
| **Integral Gain** | $K_i$ | `0.2285` | Eliminates residual steady-state offset on curved walls |
| **Derivative Gain** | $K_d$ | `0.0984` | Damps overshoots and smooths aggressive corrections |
| **Ultimate Gain** | $K_u$ | `0.5000` | Baseline critical oscillation gain |
| **Input Distance** | $d_{\text{target}}$ | `10.0 cm` | Target distance from the wall |

---

## 👥 Team & Acknowledgments

- **Team Name**: **Robot Wanderers**
- **Competition**: **PeraBots 2025**
- **Platform**: Cyberbotics Webots

*Special thanks to the organizing committee of PeraBots 2025 and the open-source robotics community for providing reference environments and simulation tools.*

---

<div align="center">
  <sub>Built with ❤️ by Team Robot Wanderers for PeraBots 2025</sub>
</div>
