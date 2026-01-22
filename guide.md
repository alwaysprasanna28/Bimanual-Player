# FR3 Hardware Workspace Setup (Ubuntu 20.04 + ROS Noetic)

This guide covers a from-scratch setup for running the bimanual trajectory
player on real FR3 hardware using libfranka and franka_ros.

## Prereqs
- Ubuntu 20.04, ROS Noetic
- Two FR3 arms reachable on Ethernet (static IPs)
- FCI licenses enabled on both robots

## 1) Install ROS Noetic and core tools
```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg2
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-rosdep python3-catkin-tools python3-vcstool build-essential
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2) Create a catkin workspace
```bash
mkdir -p ~/franka_ws/src
cd ~/franka_ws
catkin init
```

## 3) Install libfranka (from source)
```bash
sudo apt install -y cmake libpoco-dev libeigen3-dev
cd ~/franka_ws/src
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka
# Choose a version compatible with your franka_ros release; 0.9.x is common for Noetic.
git checkout 0.9.2
git submodule update --init --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build . -j$(nproc)
sudo cmake --install .
sudo ldconfig
```

## 4) Clone franka_ros (Noetic)
```bash
cd ~/franka_ws/src
git clone https://github.com/frankaemika/franka_ros.git
cd franka_ros
# Use the matching release branch/tag; for Noetic + libfranka 0.9.x:
git checkout 0.9.2
git submodule update --init --recursive
```

## 5) Install ROS dependencies
```bash
cd ~/franka_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 6) Add the bimanual trajectory package
If your repo is already cloned:
```bash
cd ~/franka_ws/src
git clone https://github.com/alwaysprasanna28/Bimanual-Player.git
```

## 7) Build the workspace
```bash
cd ~/franka_ws
catkin build
source devel/setup.bash
```

## 8) Configure dual-arm franka_ros
You need two `franka_control` nodes and two `ros_control` controllers. A common
convention is `panda1` and `panda2`.

Example robot config:
- `arm_id: panda1` with `robot_ip: 172.16.0.2`
- `arm_id: panda2` with `robot_ip: 172.16.0.3`

Configure two `joint_trajectory_controller` instances, one per arm, with names
like:
- `/left_arm_controller/follow_joint_trajectory`
- `/right_arm_controller/follow_joint_trajectory`

Ensure joint names match your controller configuration:
- `panda1_joint1..panda1_joint7`
- `panda2_joint1..panda2_joint7`

## 9) Update player config
Edit:
- `fr3_bimanual_trajectory_player/config/bimanual_player.yaml`

Set joint names to match your controllers.

## 10) Bring up hardware + controllers
Terminal 1:
```bash
source ~/franka_ws/devel/setup.bash
roslaunch franka_control franka_control.launch arm_id:=panda1 robot_ip:=172.16.0.2
```

Terminal 2:
```bash
source ~/franka_ws/devel/setup.bash
roslaunch franka_control franka_control.launch arm_id:=panda2 robot_ip:=172.16.0.3
```

Then start your controllers (example using controller_manager):
```bash
rosservice call /panda1/controller_manager/switch_controller "start_controllers: ['left_arm_controller'] stop_controllers: [] strictness: 2"
rosservice call /panda2/controller_manager/switch_controller "start_controllers: ['right_arm_controller'] stop_controllers: [] strictness: 2"
```

## 11) Run the trajectories
```bash
roslaunch fr3_bimanual_trajectory_player play_bimanual_trajectories.launch
```
