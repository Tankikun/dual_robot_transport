# Dual Robot Cooperative Transport System

A synchronized multi-robot formation control framework for transporting unattached payloads using **ROS 2 Humble**.

- **Current Architecture:** Centralized Virtual Structure (Splitter Node)  
- **Target Architecture:** Decentralized Leader–Follower (Complex Laplacian)

---

## 1. Prerequisites & Setup

### Hardware
- **Robots:** 2× TurtleBot3 Burger (Raspberry Pi 3/4)
- **Network:** Wi-Fi 6 Router (5 GHz, 80 MHz channel width)
- **Middleware:** Cyclone DDS (required for stability)

### Network Configuration (Static IPs)
Configure your router to reserve:

| Device | IP Address |
|--------|------------|
| **PC** | `192.168.1.100` |
| **Robot A (Left / Leader)** | `192.168.1.101` |
| **Robot B (Right / Follower)** | `192.168.1.102` |

### Environment Variables
Add the following to `~/.bashrc` on **all** devices (PC & Robots):

```bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share
````

---

## 2. Installation (PC / Developer Laptop)

### 1. Create Workspace

```bash
mkdir -p ~/transport_ws/src
cd ~/transport_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/dual_robot_transport.git
```

### 3. Build

```bash
cd ~/transport_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. Development Workflow (Important)

**Never edit code directly on the robots.**
Robots are *execution targets only*.

### Sync Workflow

1. Edit code on your laptop.
2. Sync code to both robots:

   ```bash
   ./scripts/sync_robots.sh
   ```
3. SSH into each robot and rebuild:

   ```bash
   ssh ubuntu@192.168.1.101
   rebuild   # alias previously configured
   ```

### Git Branching Strategy

* **`main`** – Stable, demo-ready code. *Never push broken code.*
* **`feature/decentralized`** – Development of leader–follower logic.
* **`experiment/pid-tuning`** – Temporary PID tuning branch.

**Creating a new feature branch:**


git checkout main
git pull
git checkout -b feature/my-new-feature
```

---

## 4. Usage Instructions

### A. Simulation (Gazebo)

Run the multi-robot simulation:

```bash
ros2 launch dual_robot_control multi_sim.launch.py
```

Then run the controller in another terminal:

```bash
ros2 run dual_robot_control auto_splitter
```

---

### B. Physical Robots (Centralized Mode)

1. Power on the robots and ensure they connect to **RobotLab_5G**.
2. Launch Robot A:

   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
   ```
3. Launch Robot B:

   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_1
   ```
4. Run controller on PC:

   ```bash
   ros2 run dual_robot_control auto_splitter --ros-args -p auto_set_dist:=True
   ```
5. Drive manually:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

---

### C. Navigation (Autonomous Mode)

Launch with custom wide-footprint config:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    params_file:=src/dual_robot_transport/config/dual_burger.yaml
```

---

## 5. Troubleshooting Cheat Sheet

| Issue                                | Solution                                                |
| ------------------------------------ | ------------------------------------------------------- |
| **`[RTPS_TRANSPORT_SHM Error]`**     | Run: `sudo rm -rf /dev/shm/*`                           |
| Robots move identically (no turning) | Namespace missing. Ensure `namespace:=tb3_0` / `tb3_1`. |
| “Failed connection with Devices”     | Check OpenCR USB cable and press **RESET**.             |
| Formation breaks during turn         | 1. Check `separation_len`.<br>2. Lower `max_vel_theta`. |
| `Temporary failure resolving...`     | Connect robot to online Wi-Fi using `sudo nmtui`.       |

---

```

If you'd like, I can also produce:

✅ A **PDF-ready** version  
✅ A **tight single-page version** for printing  
✅ A **developer-optimized version** with callouts (NOTES / TIPS / WARNINGS)  
✅ A **version with diagrams** (ASCII or images)

Just tell me!
```
