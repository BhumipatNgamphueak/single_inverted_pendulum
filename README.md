# Single Inverted Pendulum Control System

A **ROS 2** robotics project implementing energy-based swing-up and LQR stabilization for an underactuated inverted pendulum system in Gazebo simulation.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)

---

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Harmonic (ros_gz_sim)
- Python 3.8+
- NumPy

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/BhumipatNgamphueak/single_inverted_pendulum.git

# Navigate to workspace
cd single_inverted_pendulum

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### Make Sourcing Permanent (Optional)

To avoid sourcing the workspace in every new terminal, add this line to your `~/.bashrc`:

```bash
echo "source ~/single_inverted_pendulum/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Note:** Adjust the path if your workspace is located elsewhere. This will automatically source the workspace whenever you open a new terminal.

### Run Simulation

**Terminal 1 - Launch Gazebo simulation:**
```bash
cd single_inverted_pendulum
source install/setup.bash
ros2 launch single_inverted_pendulum_simulation simulation_single_launch.py
```

**Terminal 2 - Launch controller (wait 5-7 seconds after Gazebo starts):**
```bash
cd single_inverted_pendulum
source install/setup.bash
ros2 launch single_inverted_pendulum swingup_controller.launch.py
```

### Monitor the System

```bash
# View joint states
ros2 topic echo /joint_states

# View torque commands
ros2 topic echo /effort_controller/commands

# View controller mode
ros2 topic echo /controller_mode

# Visualize with rqt_graph
rqt_graph
```

---

## Project Overview

This is a **ROS 2** robotics project implementing a **single inverted pendulum control system** with energy-based swing-up and LQR stabilization. The system simulates an underactuated pendulum on a cart and demonstrates nonlinear control techniques for stabilizing an unstable equilibrium point.

**Key Features:**
- Energy-regulated swing-up controller with kick mechanism
- LQR stabilization for upright position
- Monolithic control architecture with integrated state machine
- Gazebo simulation with ros2_control integration
- Configurable disturbance injection for robustness testing

**Technology Stack:**
- ROS 2 Humble (Python + C++)
- Gazebo Harmonic (ros_gz_sim)
- ros2_control framework
- NumPy for numerical computations

## Repository Structure

```
single_inverted_pendulum/
├── README.md                              # This file
├── .gitignore
├── .gitattributes
└── src/                                   # ROS 2 workspace source directory
    ├── single_inverted_pendulum/          # Main control package
    ├── single_inverted_pendulum_description/  # Robot URDF/xacro files
    └── single_inverted_pendulum_simulation/   # Gazebo simulation configuration
```

## Package Organization

### 1. `single_inverted_pendulum` (Main Control Package)

**Purpose:** Implements all control algorithms and coordination logic.

**Key Directories:**
```
single_inverted_pendulum/
├── package.xml                            # Package dependencies
├── CMakeLists.txt                         # Build configuration
├── config/
│   └── single_pendulum_swingup_params.yaml  # Controller parameters
├── launch/
│   └── swingup_controller.launch.py       # Controller launch file
├── scripts/
│   └── single_pendulum_swingup_controller.py  # Main controller
└── single_inverted_pendulum/             # Python package (mostly empty)
    ├── __init__.py
    └── dummy_module.py
```

**Controller:**

**`single_pendulum_swingup_controller.py`** - Main controller implementing all control logic
- Energy-regulated swing-up with kick start mechanism
- LQR stabilization for upright position
- Integrated mode switching logic (SWING_UP ↔ STABILIZE)
- Built-in disturbance injection for robustness testing
- Single-process architecture for simple deployment

### 2. `single_inverted_pendulum_description` (Robot Description)

**Purpose:** URDF/xacro models for robot visualization and simulation.

**Key Directories:**
```
single_inverted_pendulum_description/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── display_launch.py                 # RViz visualization
├── robot/
│   └── visual/
│       ├── single_inverted_pendulum.xacro  # Main robot model
│       ├── pendulum_params.xacro         # Physical parameters
│       ├── gazebo_single.xacro           # Gazebo plugins (ros2_control)
│       ├── inertial_macros.xacro         # Inertia calculations
│       ├── robot_material.xacro          # Visual materials
│       └── invert_pendulum_URDF.urdf     # Alternative URDF
└── meshes/                               # 3D mesh files (if any)
```

**Important Files:**
- `single_inverted_pendulum.xacro` - Main entry point, includes all other xacro files
- `gazebo_single.xacro` - Defines ros2_control hardware interface for Gazebo
- `pendulum_params.xacro` - Physical parameters (masses, lengths, inertias)

### 3. `single_inverted_pendulum_simulation` (Gazebo Simulation)

**Purpose:** Gazebo world files and controller configuration for simulation.

**Key Directories:**
```
single_inverted_pendulum_simulation/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── controller_single.yaml            # ros2_control configuration
├── launch/
│   └── simulation_single_launch.py       # Gazebo + controllers
├── worlds/
│   └── empty.sdf                         # Gazebo world file
└── rviz/                                 # RViz configurations
```

**Important Files:**
- `controller_single.yaml` - Defines `joint_state_broadcaster` and `effort_controller`
- `simulation_single_launch.py` - Complete simulation setup with timed controller loading

## Control System Architecture

```
single_pendulum_swingup_controller.py
          ↓
    /joint_states (input)
          ↓
  [Mode State Machine]
          ↓
  SWING_UP → energy control
  STABILIZE → LQR control + disturbance
          ↓
    /effort_controller/commands (output)
```

**Architecture Features:**
- Single-process monolithic design
- Integrated state machine for mode switching
- Direct sensor-to-actuator control loop
- Simple deployment and debugging

## Key Configuration Parameters

Located in `src/single_inverted_pendulum/config/single_pendulum_swingup_params.yaml`:

```yaml
control_frequency: 100.0  # Control loop rate (Hz)

# Physical parameters
g: 9.81
M_arm: 0.22015507085087   # Cart/arm mass (kg)
l_arm: 0.157              # Arm length (m)
M1: 0.190                 # Pendulum mass (kg)
l1: 0.0878                # Pendulum COM distance (m)
L1: 0.14                  # Pendulum total length (m)

# LQR gains
K_theta: 2.40000000000
K_alpha: 11.24336850534
K_theta_dot: 0.96104380212
K_alpha_dot: -0.44719736055

# Swing-up parameters
energy_gain: 5.00         # Energy pumping gain
damping_gain: 0.11        # Base damping
kick_torque: 5.0          # Initial kick (Nm)
swing_up_max_torque: 10.0

# Mode switching
switch_to_lqr_angle: 0.2      # Capture zone (rad)
switch_to_lqr_velocity: 2.0   # Max velocity for switch (rad/s)
stabilization_time: 0.15      # Required stable time (s)
fall_angle_threshold: 0.8     # Fall detection (rad)

# Disturbance testing
disturbance_delay: 5.0        # Time after stabilization (s)
disturbance_duration: 0.15    # Pulse duration (s)
disturbance_torque: 0.0       # Torque magnitude (Nm)
```

## ROS 2 Topics

### Input Topics
- `/joint_states` (sensor_msgs/JointState) - Joint positions and velocities from Gazebo
  - `revolute_joint` - Cart/base position (θ)
  - `first_pendulum_joint` - Pendulum angle (α)

### Output Topics
- `/effort_controller/commands` (std_msgs/Float64MultiArray) - Torque commands
  - Index 0: Torque on revolute_joint (actuated)
  - Index 1: Torque on first_pendulum_joint (passive, usually 0)

## Development Workflow

### Building the Project

```bash
# Navigate to workspace root
cd single_inverted_pendulum

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

**Build System:**
- Uses `ament_cmake` build type (all packages)
- Python scripts installed as executables (no compilation needed)
- CMakeLists.txt handles installation of launch files, configs, and scripts

### Running Simulations

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch single_inverted_pendulum_simulation simulation_single_launch.py

# Terminal 2: Launch controller (after controllers load ~5-7 seconds)
ros2 launch single_inverted_pendulum swingup_controller.launch.py
```

**Important Timing Notes:**
- Gazebo needs ~3 seconds to initialize ros2_control
- `joint_state_broadcaster` loads at t=3s
- `effort_controller` loads at t=5s
- Start control nodes after t=5s

### Testing the Controller

```bash
# Run the controller directly (for debugging)
ros2 run single_inverted_pendulum single_pendulum_swingup_controller.py
```

### Monitoring System

```bash
# View joint states
ros2 topic echo /joint_states

# View effort commands
ros2 topic echo /effort_controller/commands

# List all active topics
ros2 topic list

# View node graph
rqt_graph
```

## Code Conventions and Style

### Python Style
- **Shebang:** All scripts start with `#!/usr/bin/env python3`
- **Docstrings:** Module-level docstrings explain purpose
- **Naming:**
  - Snake_case for functions, variables
  - PascalCase for classes
  - UPPER_CASE for constants
- **ROS 2 Nodes:** Inherit from `rclpy.node.Node`
- **Main function:** Always includes `if __name__ == '__main__':` guard

### Control Logic Patterns

**State Management:**
```python
self.current_state = np.zeros(4)  # [theta, alpha, theta_dot, alpha_dot]
```

**Angle Wrapping:** Always wrap angles to [-π, π]
```python
def _wrap_to_pi(self, angle):
    return np.arctan2(np.sin(angle), np.cos(angle))
```

**Filtering:** Low-pass filters on velocities
```python
class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
```

**Control Loop Pattern:**
```python
try:
    while rclpy.ok():
        start = time.time()
        rclpy.spin_once(self, timeout_sec=0.0)
        self.control_step()

        elapsed = time.time() - start
        if elapsed < self.control_period:
            time.sleep(self.control_period - elapsed)
except KeyboardInterrupt:
    pass
finally:
    # Send zero torques
    self.publish_zero_torques()
```

### Safety Practices

1. **Always publish zero torques on shutdown:**
   ```python
   finally:
       msg = Float64MultiArray()
       msg.data = [0.0, 0.0]
       self.effort_pub.publish(msg)
   ```

2. **Torque saturation:**
   ```python
   tau = np.clip(tau, -max_torque, max_torque)
   ```

3. **State machine safety:**
   - Fall detection → automatic switch to SWING_UP
   - Stabilization counters prevent chattering

## Important Implementation Details

### 1. Coordinate System and Sign Conventions

- **θ (theta):** Cart/base position (revolute_joint)
  - Positive: counterclockwise rotation
  - Measured in radians
  - Wrapped to [-π, π]

- **α (alpha):** Pendulum angle (first_pendulum_joint)
  - α = 0: Hanging down (stable equilibrium)
  - α = π: Upright (unstable equilibrium, target)
  - α ≈ 6.28 (2π): Also hanging down (wrapped)

### 2. Swing-Up Control Law

```python
# Energy error
E_error = E_desired - E_current

# Energy regulation (reduce pumping near target)
energy_ratio = E_current / E_desired
if energy_ratio > 0.9:
    regulation_factor = max(0.0, (1.0 - energy_ratio) / 0.1)
else:
    regulation_factor = 1.0

# Control signal
control_term = alpha_dot * cos(alpha)
u_energy = pump_sign * energy_gain * E_error * control_term * regulation_factor
u_damping = -damping_gain * theta_dot
tau = u_energy + u_damping
```

**Key Insight:** The `alpha_dot * cos(alpha)` term ensures energy is pumped in phase with pendulum motion.

### 3. LQR Stabilization

```python
# State error (relative to setpoint)
theta_error = wrap_to_pi(theta - theta_setpoint)
alpha_error = wrap_to_pi(alpha - pi)
x_error = [theta_error, alpha_error, theta_dot, alpha_dot]

# LQR control law
tau = -K @ x_error

# K = [K_theta, K_alpha, K_theta_dot, K_alpha_dot]
```

**Setpoint Tracking:** The system captures `theta_setpoint` when switching to STABILIZE mode, preventing drift.

### 4. Mode Switching Logic

**Condition to switch SWING_UP → STABILIZE:**
```python
alpha_dist = abs(wrap_to_pi(alpha - pi)) < 0.2  # Within 0.2 rad of upright
velocity_ok = abs(alpha_dot) < 2.0  # Slow enough
stable_time = counter >= (0.15s * 100Hz)  # Stable for 15 samples
```

**Condition to switch STABILIZE → SWING_UP:**
```python
alpha_dist = abs(wrap_to_pi(alpha - pi)) > 0.8  # Fallen too far
```

### 5. Kick Mechanism

Initial impulse to break stiction and start motion:
```python
if not kick_applied:
    if kick_counter < kick_duration:  # 20 samples = 0.2s
        kick_counter += 1
        return kick_torque  # 5.0 Nm
    else:
        kick_applied = True
```

## Additional Resources

### ROS 2 Documentation
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [ros2_control](https://control.ros.org/humble/index.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)

### Control Theory
- Energy-based control for underactuated systems
- Linear Quadratic Regulator (LQR) theory
- Nonlinear dynamics and stabilization

### Related Files
- Check `src/single_inverted_pendulum/scripts/` for implementation examples
- Review YAML configs before changing controller parameters
- Examine launch files for system startup sequence

---
