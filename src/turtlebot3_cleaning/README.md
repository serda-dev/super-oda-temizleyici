# TurtleBot3 Cleaning Mission

A ROS1 Noetic project for a TurtleBot3 robot performing a house-cleaning mission with room verification using QR codes.

## Prerequisites
- ROS Noetic
- TurtleBot3 Packages
- Gazebo
- OpenCV (`libopencv-dev`)
- Python `qrcode` (`pip install qrcode[pil]`)
- **Explore Lite** (`sudo apt install ros-noetic-explore-lite`)
- **SLAM Toolbox** (`sudo apt install ros-noetic-slam-toolbox`)

## Environment Setup
Before running any commands, you must set the robot model. Add this to your `~/.bashrc` to make it permanent.
```bash
export TURTLEBOT3_MODEL=burger
```

source devel/setup.bash


## Quick Fixes / Reset
If the system crashes or Gazebo fails to launch (Exit Code 255):
```bash
killall -9 rosmaster gzserver gzclient rosout
```

## Phase 1: Setup & Mapping
This phase automatically explores the environment and builds the map.

1.  **Launch Simulation** (Terminal 1 - Keep Running):
    ```bash
    roslaunch turtlebot3_cleaning sim.launch
    ```
    *This spawns the house, the robot, and the QR code posters.*

2.  **Start Autonomous Mapping** (Terminal 2):
    ```bash
    roslaunch turtlebot3_cleaning autonomous_mapping.launch
    ```
    *The robot will explore the house automatically.*

3. **Eğer ki RViz'i görmek istersen** 
   ```bash
   rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
   ```

4.  **Save the Map** (Once exploration is complete):
    ```bash
    rosrun map_server map_saver -f ~/catkin_ws/src/turtlebot3_cleaning/maps/map
    ```

## Phase 2: Mission Execution
Once the map is saved, you can run the actual cleaning mission.

1.  **Stop** `autonomous_mapping.launch` in Terminal 2 (Ctrl+C).
2.  **Keep** `sim.launch` running in Terminal 1 (or restart it to reset robot pose).
3.  **Run the Mission** (Terminal 2):
    ```bash
    roslaunch turtlebot3_cleaning task_manager.launch
    ```

    This will:
    - Load your saved map.
    - Start the Task Manager and QR Reader.
    - Navigate to each room defined in `mission.yaml`.
    - Perform cleaning and QR verification.

## Configuration
- **Mission Goals**: Edit `config/mission.yaml` to adjust room coordinates.
- **QR Codes**: Generated automatically in `models/materials/textures`.

## Troubleshooting
- **Robot stuck / Recovery behaviors**: Check if `mission.yaml` coordinates match your map. Use RViz "2D Nav Goal" to verify reachable points.
- **Localization issues**: If the robot pose in RViz doesn't match Gazebo, use the **"2D Pose Estimate"** tool in RViz to manually align the robot.
- **Resource Not Found**: Ensure you have all dependencies install (e.g., `ros-noetic-turtlebot3-navigation`, `ros-noetic-turtlebot3-slam`).
