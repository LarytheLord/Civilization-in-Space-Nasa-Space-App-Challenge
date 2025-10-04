#!/usr/bin/env python3
"""
Test script to verify the new habitat site selection modules
"""

import os
import sys

def test_new_modules():
    print("Testing new LunaBot habitat site selection modules...")
    
    # Define the expected new files
    new_files = [
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/src/lunabot_navigation/scripts/habitat_site_analyzer.py",
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/src/lunabot_navigation/scripts/websocket_bridge.py",
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/src/lunabot_navigation/scripts/cost_estimator.py"
    ]
    
    # Check if files exist
    all_found = True
    for file_path in new_files:
        if os.path.exists(file_path):
            print(f"[OK] Found: {os.path.basename(file_path)}")
        else:
            print(f"[ERROR] Missing: {os.path.basename(file_path)}")
            all_found = False
    
    # Check if launch file was updated
    launch_file = "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/src/lunabot_navigation/launch/lunabot_complete.launch.py"
    if os.path.exists(launch_file):
        try:
            with open(launch_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'habitat_site_analyzer' in content and 'websocket_bridge' in content:
                    print("[OK] Launch file updated with new nodes")
                else:
                    print("[ERROR] Launch file not properly updated")
                    all_found = False
        except UnicodeDecodeError:
            # Try with different encoding
            with open(launch_file, 'r', encoding='latin-1') as f:
                content = f.read()
                if 'habitat_site_analyzer' in content and 'websocket_bridge' in content:
                    print("[OK] Launch file updated with new nodes")
                else:
                    print("[ERROR] Launch file not properly updated")
                    all_found = False
    else:
        print("[ERROR] Launch file not found")
        all_found = False
    
    # Check if CMakeLists was updated
    cmake_file = "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/src/lunabot_navigation/CMakeLists.txt"
    if os.path.exists(cmake_file):
        try:
            with open(cmake_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'habitat_site_analyzer.py' in content and 'websocket_bridge.py' in content:
                    print("[OK] CMakeLists.txt updated with new scripts")
                else:
                    print("[ERROR] CMakeLists.txt not properly updated")
                    all_found = False
        except UnicodeDecodeError:
            # Try with different encoding
            with open(cmake_file, 'r', encoding='latin-1') as f:
                content = f.read()
                if 'habitat_site_analyzer.py' in content and 'websocket_bridge.py' in content:
                    print("[OK] CMakeLists.txt updated with new scripts")
                else:
                    print("[ERROR] CMakeLists.txt not properly updated")
                    all_found = False
    else:
        print("[ERROR] CMakeLists.txt not found")
        all_found = False
    
    # Check if dependencies were updated
    deps_file = "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/lunabot_ws/install_dependencies.py"
    if os.path.exists(deps_file):
        try:
            with open(deps_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'websockets' in content and 'aiohttp' in content:
                    print("[OK] Dependencies updated with WebSocket packages")
                else:
                    print("[ERROR] Dependencies not properly updated")
                    all_found = False
        except UnicodeDecodeError:
            # Try with different encoding
            with open(deps_file, 'r', encoding='latin-1') as f:
                content = f.read()
                if 'websockets' in content and 'aiohttp' in content:
                    print("[OK] Dependencies updated with WebSocket packages")
                else:
                    print("[ERROR] Dependencies not properly updated")
                    all_found = False
    else:
        print("[ERROR] Dependencies file not found")
        all_found = False
    
    # Check frontend files
    frontend_files = [
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/frontend/package.json",
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/frontend/src/App.tsx",
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/frontend/src/components/SiteAnalysisPanel.tsx",
        "C:/Users/A R Khan/OneDrive/Documents/Nasa/robot/frontend/src/hooks/useWebSocket.ts"
    ]
    
    for file_path in frontend_files:
        if os.path.exists(file_path):
            print(f"[OK] Frontend file found: {os.path.basename(file_path)}")
        else:
            print(f"[ERROR] Missing frontend file: {os.path.basename(file_path)}")
            all_found = False
    
    print("\n" + "="*50)
    if all_found:
        print("[SUCCESS] All new modules successfully created!")
        print("[SUCCESS] The habitat site selection system has been integrated")
        print("[SUCCESS] Ready for ROS2 build and testing")
    else:
        print("[ERROR] Some components are missing. Please check the implementation.")
    
    return all_found

if __name__ == "__main__":
    success = test_new_modules()
    sys.exit(0 if success else 1)