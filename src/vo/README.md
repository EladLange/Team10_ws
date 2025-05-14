# Velocity Obstacle (VO) Node

This package implements a Velocity Obstacle (VO) algorithm for collision avoidance in dynamic environments. The VO node subscribes to ego vehicle and obstacle information, computes safe velocities, and publishes velocity commands and visualization markers.

## Features

- Velocity Obstacle algorithm for collision avoidance
- Visualization of velocity obstacles in RViz
- Debug information and visualization
- Configurable debug levels

## Topics

### Subscribed Topics

- `/ego_pose` (geometry_msgs/PoseStamped): Ego vehicle position and orientation
- `/ego_velocity` (geometry_msgs/TwistStamped): Ego vehicle velocity
- `/obstacles` (visualization_msgs/MarkerArray): Obstacle positions
- `/obstacles_velocity` (visualization_msgs/MarkerArray): Obstacle velocities

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Computed safe velocity command
- `/vo_marker_array` (visualization_msgs/MarkerArray): Visualization markers for RViz
- `/vo_debug` (std_msgs/String): Debug information

## Parameters

- `debug_level` (int, default: 0): Debug verbosity level
  - 0: Info level (minimal output)
  - 1: Debug level (more detailed output)
  - 2: Verbose level (maximum output)

## Usage

### Running the VO Node

To run the VO node with the environment simulation:

```bash
ros2 launch vo vo_system.launch.py
```

To run with a specific debug level:

```bash
ros2 launch vo vo_system.launch.py debug_level:=2
```

### Running the VO Node in Debug Mode

For debugging purposes, you can run just the VO node with:

```bash
ros2 launch vo vo_debug.launch.py debug_level:=2
```

## Visualization in RViz

To visualize the VO algorithm in RViz:

1. Start RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. Add the following displays:
   - MarkerArray: Set the topic to `/vo_marker_array`
   - MarkerArray: Set the topic to `/visualization_marker_array` (for environment visualization)

3. Set the Fixed Frame to `map`

## Debugging

To view debug messages:

```bash
ros2 topic echo /vo_debug
```

To view the computed velocity commands:

```bash
ros2 topic echo /cmd_vel
```

## Implementation Details

The VO node implements the following components:

1. **Ego Vehicle Tracking**: Subscribes to ego vehicle pose and velocity
2. **Obstacle Tracking**: Subscribes to obstacle poses and velocities
3. **Velocity Obstacle Computation**: Calculates velocity obstacles for each obstacle
4. **Velocity Selection**: Selects the best velocity that avoids collisions
5. **Visualization**: Publishes markers for visualization in RViz

## Troubleshooting

If you encounter issues:

1. Check that all required topics are being published:
   ```bash
   ros2 topic list
   ```

2. Verify that the ego pose and obstacles are being received:
   ```bash
   ros2 topic echo /ego_pose
   ros2 topic echo /obstacles
   ```

3. Increase the debug level to get more detailed information:
   ```bash
   ros2 launch vo vo_system.launch.py debug_level:=2
   ```
