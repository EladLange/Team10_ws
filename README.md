# Team 10 workspace

## Open vo in RVIZ:
I order to open the vo simulation in RVIZ, the next steps required:
1. in Team10_ws:
```bash
cd /mnt/c/repos/Team10_ws
rviz2
```
2. once the Rviz is open:
    * Uncheck th grid box
    * Set the fixed frame to *map*
    * Add Marker display:
        * Set the topic to: */visualization_marker*
    * Add Marker Array display:
        * Set the topic to */visualization_marker_array*
3.  in the workspace:
```bash
 source install/setup file
 ```
4. in Team10_ws:
```bash
colcon build && ros2 run vo vo_simulation_node 
```