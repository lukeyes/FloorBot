#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# ==========================================
# 1. SYSTEM UPDATE & DEPENDENCIES
# ==========================================
echo "--- [1/4] Updating System & Installing ROS 2 Kilted Dependencies ---"
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y python3-pip python3-colcon-common-extensions git build-essential wget curl

# Install ROS 2 Kilted Packages
sudo apt install -y \
  ros-kilted-joy \
  ros-kilted-teleop-twist-joy \
  ros-kilted-navigation2 \
  ros-kilted-nav2-bringup \
  ros-kilted-xacro \
  ros-kilted-depthimage-to-laserscan \
  ros-kilted-robot-localization \
  ros-kilted-tf2-tools \
  ros-kilted-tf-transformations

# Install Python Libraries (with break-system-packages for Ubuntu 24.04)
pip3 install ultralytics numpy pyserial transforms3d --break-system-packages

# ==========================================
# 2. GOOGLE CORAL SETUP
# ==========================================
echo "--- [2/4] Installing Google Coral USB Drivers ---"
# Add Google Coral Repo
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

# Install Edge TPU Runtime
sudo apt update
sudo apt install -y libedgetpu1-std

# ==========================================
# 3. WORKSPACE & SOURCE BUILDS
# ==========================================
echo "--- [3/4] Setting up Workspace & Cloning Drivers ---"
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone Create 1 Driver (using humble branch as base compatibility)
if [ ! -d "create_robot" ]; then
    git clone -b humble https://github.com/AutonomyLab/create_robot.git
else
    echo ">> create_robot already exists. Skipping clone."
fi

# Clone RealSense (if binaries are missing, otherwise apt handles it)
# git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git

# Create Custom Package Skeleton ONLY if it doesn't exist
if [ ! -d "semantic_nav" ]; then
    echo ">> Creating semantic_nav package..."
    ros2 pkg create --build-type ament_python semantic_nav --dependencies rclpy sensor_msgs nav_msgs std_msgs
    # Create folder structure for your files
    mkdir -p semantic_nav/config
    mkdir -p semantic_nav/launch
else
    echo ">> semantic_nav package already exists. Skipping creation."
fi

# ==========================================
# 4. BUILD & PERMISSIONS
# ==========================================
echo "--- [4/4] Building Workspace & Setting Permissions ---"
cd ~/robot_ws
colcon build --symlink-install

# Add user to dialout for Serial/USB access
if ! groups $USER | grep &>/dev/null '\bdialout\b'; then
    sudo usermod -a -G dialout $USER
    echo ">> User added to 'dialout' group. You MUST reboot or logout/login for this to take effect!"
fi

# ==========================================
# FINAL INSTRUCTIONS
# ==========================================
echo ""
echo "=========================================================="
echo "   SYSTEM SETUP COMPLETE!"
echo "=========================================================="
echo "NEXT STEPS:"
echo "1. Place your 'vision_processor.py' and 'temporal_mapper.py' in:"
echo "   ~/robot_ws/src/semantic_nav/semantic_nav/"
echo "2. Place your 'ekf.yaml' in:"
echo "   ~/robot_ws/src/semantic_nav/config/"
echo "3. Place your 'robot_bringup.launch.py' in:"
echo "   ~/robot_ws/src/semantic_nav/launch/"
echo "4. Make sure your Python scripts are executable: chmod +x *.py"
echo "5. Rebuild: cd ~/robot_ws && colcon build --symlink-install"
echo "6. Source: source ~/robot_ws/install/setup.bash"
echo "=========================================================="