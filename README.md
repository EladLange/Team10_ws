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
#
#
## open map in rviz
### Terminal 1:
```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/yonatan/Desktop/Team10_ws/free_area_map.yaml
```
change the path to the correct one.

### Terminal 2:
```
ros2 lifecycle set map_server configure
```

after getting this output:
```
yonatan@yonatan:~$ ros2 lifecycle set map_server configure
Transitioning successful
```
run thi command:
```
ros2 lifecycle set map_server activate
```
you should get this output:
```
yonatan@yonatan:~$ ros2 lifecycle set map_server activate
Transitioning successful
```
### Terminal 3:
open rviz:
```
rviz2
```
In rviz adding map.

## pure pursuit in rviz

### Terminal 1:
Go to the correct location:
```
cd /path/to/Team10_ws/

```
RUN:
```
ros2 run purepursuit_new pure_pursuit_node
```
### Terminal 2:
open Rviz

RUN:
```
rviz2
```
add -> By topic:
```
1. /desired_path
    path

2. /trajectory
    path

3. /vehicle_marker
    Marker
```