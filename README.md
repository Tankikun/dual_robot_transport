# Dual Robot Cooperative Transport

## Installation
1. Create a workspace:
   `mkdir -p ~/transport_ws/src`
   `cd ~/transport_ws/src`
2. Clone this repo:
   `git clone https://github.com/YOUR_USERNAME/dual_robot_transport.git`
3. Build:
   `cd ~/transport_ws`
   `colcon build --symlink-install`
   `source install/setup.bash`

## Usage
### 1. Simulation
`ros2 launch dual_robot_control multi_sim.launch.py`

### 2. Physical Robots
1. Connect robots to Wi-Fi.
2. Launch hardware on robots (`go` alias).
3. Run Splitter on PC:
   `ros2 run dual_robot_control smart_splitter`

### 3. Navigation
Run Nav2 with our custom configuration:
`ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=src/dual_robot_transport/config/dual_burger.yaml`
