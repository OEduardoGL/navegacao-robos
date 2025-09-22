Classic ROS2 + Gazebo (Classic) TurtleBot3 Bug Navigation

This is a clean, self‑contained setup using ROS 2 Humble + Gazebo Classic inside Docker. It runs TurtleBot3 (waffle) with working LIDAR and two algorithms: Bug1 and TangentBug. A custom world with 3 obstacles (one concave) is included.

Quick start (Docker Compose)
- Requirements: Docker + Docker Compose + host X11 enabled (Linux)
- One‑time: `xhost +local:`
- Launch (builds automatically):
  - `cd classic_nav`
  - `docker compose up --build`

In the container (new shell opens)
- Bug1: `ros2 launch tb3_bug_nav_classic bug1_classic.launch.py`
- Tangent-Bug: `ros2 launch tb3_bug_nav_classic tangent_classic.launch.py`

Record rosbag
- `ros2 bag record -o bug_run /odom /scan /cmd_vel /bug_state`

Files
- docker-compose.yml, Dockerfile, entrypoint.sh
- Workspace: `ws/src/tb3_bug_nav_classic` (code + launch + world)
