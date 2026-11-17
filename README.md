# ARM Lab 05 — Robot poszukujący

## Instalacja

Doinstalować `aruco_ros`:

```bash
apt install -y ros-jazzy-aruco-ros
```

## Uruchomienie

**Terminal 1 — Gazebo i Nav2**

```bash
ros2 launch arm05_sim world.launch.py
```

**Terminal 2 — RViz2**

```bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

W RViz2 można dodać tematy `MarkerArray`:
- `/map_points`
- `/aruco/map_markers`

**Terminal 3 — Węzły projektu**

```bash
ros2 launch aruco_explorer aruco_nodes.launch.py
```

**Terminal 4 — Rozpoczęcie wyszukiwania przez akcję ROS 2**

```bash
ros2 action send_goal /find_aruco aruco_explorer_interfaces/action/FindAruco "{start: true}" --feedback
```
