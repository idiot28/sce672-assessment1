#!/usr/bin/env bash
# ==============================================================================
# rm_setup.sh - Environment Loader for RoboMaster EP Development on Ubuntu
# Course: SCE 672 - Advanced Robotics (KFUPM)
# ==============================================================================

echo "======================================================================"
echo "[*] Loading RoboMaster EP Development Environment"
echo "======================================================================"

# 1. Source Installed ROS 2 Distribution (Auto-detect Humble or Jazzy)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[+] Sourced ROS 2 Humble (/opt/ros/humble/setup.bash)"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[+] Sourced ROS 2 Jazzy (/opt/ros/jazzy/setup.bash)"
else
    echo "[!] Warning: No ROS 2 installation found in /opt/ros."
fi

# 2. Activate Python Virtual Environment (if exists)
VENV_PATH="${HOME}/rm_venv"
if [ -f "${VENV_PATH}/bin/activate" ]; then
    source "${VENV_PATH}/bin/activate"
    echo "[+] Activated Python Virtual Environment: ${VENV_PATH}"
fi

# 3. Source RoboMaster ROS 2 Workspace Overlay (if built)
ROS2_WS="${HOME}/ros2_ws"
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
    echo "[+] Sourced RoboMaster ROS 2 workspace (${ROS2_WS}/install/setup.bash)"
fi

# 4. DDS & Network Configuration
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 5. Diagnostic Summary
echo "----------------------------------------------------------------------"
echo "Active Python: $(which python3)"
echo "Python Version: $(python3 --version 2>&1)"
python3 -c "import numpy; print('Active NumPy Version:', numpy.__version__)" 2>/dev/null || true
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "======================================================================"
echo "[+] Environment ready. You can now launch robomaster_ros or ep_teleop_sdk.py"
