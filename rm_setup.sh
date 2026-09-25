#!/usr/bin/env bash
# ==============================================================================
# rm_setup.sh - Environment Loader for RoboMaster EP Development on Ubuntu 24.04
# Course: SCE 672 - Advanced Robotics (KFUPM)
# Assessment: Term Project Assessment 1
# Instructor: Prof. Aamer Iqbal Bhatti
# ==============================================================================
# This script provisions and isolates the runtime environment for:
# 1. ROS 2 Jazzy Jalisco middleware
# 2. RoboMaster ROS Driver workspace (~/ros2_ws)
# 3. Isolated Python 3.12 Virtual Environment (~/rm_venv) adhering to PEP 668
# 4. Prevention of NumPy 2.x ABI conflicts via PYTHONNOUSERSITE and explicit pinning
# ==============================================================================

echo "======================================================================"
echo "[*] Loading RoboMaster EP Development Environment (Ubuntu 24.04 / Jazzy)"
echo "======================================================================"

# 1. Enforce PEP 668 & NumPy 2.x ABI Protection
# Prevents Python from discovering user-site packages in ~/.local/lib/python3.12
# which could introduce incompatible NumPy 2.x binaries into compiled ROS 2 nodes.
export PYTHONNOUSERSITE=1
echo "[+] PYTHONNOUSERSITE set to 1 (User-site package bleeding prevented)"

# 2. Source ROS 2 Jazzy Jalisco Base Environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[+] Sourced ROS 2 Jazzy Jalisco (/opt/ros/jazzy/setup.bash)"
else
    echo "[!] Warning: /opt/ros/jazzy/setup.bash not found. Ensure ROS 2 Jazzy is installed."
fi

# 3. Activate Isolated Python Virtual Environment
VENV_PATH="${HOME}/rm_venv"
if [ -f "${VENV_PATH}/bin/activate" ]; then
    source "${VENV_PATH}/bin/activate"
    echo "[+] Activated Python Virtual Environment: ${VENV_PATH}"
else
    echo "[!] Warning: Virtual environment at ${VENV_PATH} not found."
    echo "    Create it via: python3 -m venv --system-site-packages ~/rm_venv"
fi

# 4. Configure PYTHONPATH
# Prioritize virtual environment packages (numpy<2, robomaster SDK) while retaining ROS 2 rclpy
if [ -d "${VENV_PATH}/lib/python3.12/site-packages" ]; then
    export PYTHONPATH="${VENV_PATH}/lib/python3.12/site-packages:${PYTHONPATH}"
    echo "[+] Prepending virtual environment site-packages to PYTHONPATH"
fi

# 5. Source RoboMaster ROS 2 Workspace Overlay
ROS2_WS="${HOME}/ros2_ws"
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
    echo "[+] Sourced RoboMaster ROS 2 workspace (${ROS2_WS}/install/setup.bash)"
else
    echo "[!] Warning: ${ROS2_WS}/install/setup.bash not found. Build via 'colcon build --symlink-install'."
fi

# 6. DDS & Network Configuration for RoboMaster AP/RNDIS Communication
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
echo "[+] Configured ROS_DOMAIN_ID=0 and ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

# 7. Print Diagnostic Summary
echo "----------------------------------------------------------------------"
echo "Active Python: $(which python3)"
echo "Python Version: $(python3 --version 2>&1)"
python3 -c "import numpy; print('Active NumPy Version:', numpy.__version__)" 2>/dev/null || echo "NumPy: Not loaded yet"
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "======================================================================"
echo "[+] Environment ready. You can now launch robomaster_ros or ep_teleop_sdk.py"
