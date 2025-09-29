
# Autonomous Mapping and Exploration

This repository contains a **ROS2 node** for autonomous robot mapping and exploration in unknown environments.  
The node uses **frontier-based exploration** on an occupancy grid map to generate navigation waypoints and guide the robot until the environment is fully explored.

## Features
-  Subscribes to `/map` and `/odom` for occupancy grid and odometry data  
-  Detects **frontier cells** (boundaries between explored and unexplored space)  
-  Uses **A\*** search to evaluate reachability of frontier candidates  
-  Scores candidates using **information gain**  
-  Publishes multiple global waypoints on `/global_waypoints`  
-  Signals when exploration is complete via `/stop_robot`  
-  Publishes planned waypoints as a `Path` on `/global_path` for visualization  

## Topics
- **Subscribed**
  - `/map` (`nav_msgs/OccupancyGrid`) – Occupancy grid map  
  - `/odom` (`nav_msgs/Odometry`) – Robot odometry  

- **Published**
  - `/global_waypoints` (`std_msgs/Float64MultiArray`) – Exploration waypoints  
  - `/stop_robot` (`std_msgs/Bool`) – Stop signal when exploration is done  
  - `/global_path` (`nav_msgs/Path`) – Frontier waypoints for RViz visualization  

## Parameters
- `stop_threshold` (float) – Fraction of explored map before stopping (default: `0.8`)  
- `min_frontier_size` (int) – Minimum frontier cluster size (default: `5`)  
- `info_radius` (float) – Radius around candidate goal for information gain (default: `2.0`)  
- `replan_period` (float) – Time (seconds) between exploration updates (default: `5.0`)  

