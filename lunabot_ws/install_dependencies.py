#!/usr/bin/env python3
"""
LunaBot Dependencies Installation Script
Installs all required ROS2 packages and Python dependencies for the LunaBot project
"""

import subprocess
import sys
import os

def run_command(command, description):
    """Run a command and handle errors"""
    print(f"\n🚀 {description}")
    print(f"Running: {command}")
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        print(f"✅ Success: {description}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Error in {description}:")
        print(f"Command: {command}")
        print(f"Return code: {e.returncode}")
        print(f"Output: {e.stdout}")
        print(f"Error: {e.stderr}")
        return False

def main():
    print("LunaBot Project Setup - Installing Dependencies")
    print("=" * 50)
    
    # Check if ROS2 is installed
    if not os.environ.get('ROS_DISTRO'):
        print("❌ ROS2 not found! Please install ROS2 first.")
        print("Visit: https://docs.ros.org/en/humble/Installation.html")
        sys.exit(1)
    
    ros_distro = os.environ['ROS_DISTRO']
    print(f"📦 Detected ROS2 Distribution: {ros_distro}")
    
    # ROS2 packages to install
    ros_packages = [
        "ros-{}-navigation2",
        "ros-{}-nav2-bringup", 
        "ros-{}-slam-toolbox",
        "ros-{}-cartographer-ros",
        "ros-{}-robot-localization",
        "ros-{}-webots-ros2",
        "ros-{}-joint-state-publisher",
        "ros-{}-robot-state-publisher",
        "ros-{}-rviz2",
        "ros-{}-xacro",
        "ros-{}-pointcloud-to-laserscan",
        "ros-{}-laser-geometry"
    ]
    
    # Format package names with ROS distro
    formatted_packages = [pkg.format(ros_distro) for pkg in ros_packages]
    
    # Install ROS2 packages
    for package in formatted_packages:
        if not run_command(f"sudo apt install -y {package}", f"Installing {package}"):
            print(f"⚠️  Warning: Failed to install {package}, continuing...")
    
    # Python dependencies
    python_deps = [
        "numpy",
        "scipy", 
        "opencv-python",
        "matplotlib",
        "scikit-learn",
        "pandas",
        "pyyaml",
        "transforms3d",
        "fastapi",
        "uvicorn",
        "websockets",
        "aiohttp"
    ]
    
    # Additional Webots installation note
    print("\n📋 Additional Setup Required:")
    print("1. Download and install Webots from: https://cyberbotics.com/")
    print("2. Add Webots to your PATH environment variable")
    print("3. Install Webots ROS2 interface:")
    print("   sudo apt install ros-{}-webots-ros2".format(ros_distro))
    print("   OR pip3 install webots-ros2-driver")
    
    # Install Python packages
    pip_command = f"pip3 install {' '.join(python_deps)}"
    run_command(pip_command, "Installing Python dependencies")
    
    # Build the workspace
    print("\n🔨 Building ROS2 workspace...")
    os.chdir("lunabot_ws")
    
    if run_command("colcon build --symlink-install", "Building workspace"):
        print("\n✅ Setup completed successfully!")
        print("\n📋 Next steps:")
        print("1. Source the workspace: source install/setup.bash")
        print("2. Launch simulation: ros2 launch lunabot_navigation lunabot_simulation.launch.py")
        print("3. Start navigation: ros2 launch lunabot_navigation navigation.launch.py")
    else:
        print("\n❌ Build failed. Please check the errors above.")

if __name__ == "__main__":
    main()
